#include "hitscan.h"
#include "Animations.h"
#include "Resolver.h"
#include "../SDK/ConVar.h"
#include "../Logger.h"

#include "../SDK/GameEvent.h"
#include "../Vector2D.hpp"
#include <DirectXMath.h>
#include <algorithm>
#include <cmath>
#include <vector>
#include <tuple>
#include "Backtrack.h"
std::deque<Resolver::SnapShot> snapshots;
static std::array<Animations::Players, 65> players{};
extern UserCmd* cmd;
bool resolver = true;
bool occlusion = false;

void Resolver::CmdGrabber1(UserCmd* cmd1)
{
    cmd = cmd1;
}

void Resolver::reset() noexcept
{
    snapshots.clear();
}

void Resolver::saveRecord(int playerIndex, float playerSimulationTime) noexcept
{
    const auto entity = interfaces->entityList->getEntity(playerIndex);
    const auto player = Animations::getPlayer(playerIndex);
    if (!player.gotMatrix || !entity)
        return;

    SnapShot snapshot;
    snapshot.player = player;
    snapshot.playerIndex = playerIndex;
    snapshot.eyePosition = localPlayer->getEyePosition();
    snapshot.model = entity->getModel();

    if (player.simulationTime >= playerSimulationTime - 0.001f && player.simulationTime <= playerSimulationTime + 0.001f)
    {
        snapshots.push_back(snapshot);
        return;
    }

    for (int i = 0; i < static_cast<int>(player.backtrackRecords.size()); i++)
    {
        if (player.backtrackRecords.at(i).simulationTime >= playerSimulationTime - 0.001f && player.backtrackRecords.at(i).simulationTime <= playerSimulationTime + 0.001f)
        {
            snapshot.backtrackRecord = i;
            snapshots.push_back(snapshot);
            return;
        }
    }
}

void Resolver::getEvent(GameEvent* event) noexcept
{
    if (!event || !localPlayer || interfaces->engine->isHLTV())
        return;

    switch (fnv::hashRuntime(event->getName())) {
    case fnv::hash("round_start"):
    {
        //Reset all
        auto players = Animations::setPlayers();
        if (players->empty())
            break;

        for (int i = 0; i < static_cast<int>(players->size()); i++)
        {
            players->at(i).misses = 0;
        }
        snapshots.clear();
        break;
    }
    case fnv::hash("player_death"):
    {
        //Reset player
        const auto playerId = event->getInt("userid");
        if (playerId == localPlayer->getUserId())
            break;

        const auto index = interfaces->engine->getPlayerFromUserID(playerId);
        Animations::setPlayer(index)->misses = 0;
        break;
    }
    case fnv::hash("player_hurt"):
    {
        if (snapshots.empty())
            break;

        if (event->getInt("attacker") != localPlayer->getUserId())
            break;

        const auto hitgroup = event->getInt("hitgroup");
        if (hitgroup < HitGroup::Head || hitgroup > HitGroup::RightLeg)
            break;

        snapshots.pop_front(); //Hit somebody so dont calculate
        break;
    }
    case fnv::hash("bullet_impact"):
    {
        if (snapshots.empty())
            break;

        if (event->getInt("userid") != localPlayer->getUserId())
            break;

        auto& snapshot = snapshots.front();

        if (!snapshot.gotImpact)
        {
            snapshot.time = memory->globalVars->serverTime();
            snapshot.bulletImpact = Vector{ event->getFloat("x"), event->getFloat("y"), event->getFloat("z") };
            snapshot.gotImpact = true;
        }
        break;
    }
    default:
        break;
    }
    if (!resolver)
        snapshots.clear();
}

float get_backward_side(Entity* entity) {
    if (!entity->isAlive())
        return -1.f;
    const float result = Helpers::angleDiff(localPlayer->origin().y, entity->origin().y);
    return result;
}

float get_angle(Entity* entity) {
    return Helpers::angleNormalize(entity->eyeAngles().y);
}

float get_forward_yaw(Entity* entity) {
    return Helpers::angleNormalize(get_backward_side(entity) - 180.f);
}

Vector calcAngle(Vector source, Vector entityPos) {
    Vector delta = {};
    delta.x = source.x - entityPos.x;
    delta.y = source.y - entityPos.y;
    delta.z = source.z - entityPos.z;
    Vector angles = {};
    Vector viewangles = cmd->viewangles;
    angles.x = Helpers::rad2deg(atan(delta.z / hypot(delta.x, delta.y))) - viewangles.x;
    angles.y = Helpers::rad2deg(atan(delta.y / delta.x)) - viewangles.y;
    angles.z = 0;
    if (delta.x >= 0.f)
        angles.y += 180;

    return angles;
}

float build_server_abs_yaw(Entity* entity, const float angle)
{
    float m_fl_goal_feet_yaw = 0;
     int sidecheck = 0;
    Resolver::detect_side(entity, &sidecheck);

    m_fl_goal_feet_yaw = entity->eyeAngles().y + (entity->getMaxDesyncAngle() * sidecheck);
    //abs_yaw = player->angles().y + desync * side;
    //entity->eyeAngles().y -player->angles().y
    //entity->getMaxDesyncAngle -desync
    
    return Helpers::normalizeYaw(m_fl_goal_feet_yaw);

}

void Resolver::detect_side(Entity* entity, int* side) {
    /* externals */
    Vector forward{};
    Vector right{};
    Vector up{};
    Trace tr;
    Helpers::AngleVectors(Vector(0, get_backward_side(entity), 0), &forward, &right, &up);
    /* filtering */

    const Vector src_3d = entity->getEyePosition();
    const Vector dst_3d = src_3d + forward * 384;

    /* back engine tracers */
    interfaces->engineTrace->traceRay({ src_3d, dst_3d }, MASK_SHOT, { entity }, tr);
    float back_two = (tr.endpos - tr.startpos).length();

    /* right engine tracers */
    interfaces->engineTrace->traceRay(Ray(src_3d + right * 35, dst_3d + right * 35), MASK_SHOT, { entity }, tr);
    float right_two = (tr.endpos - tr.startpos).length();

    /* left engine tracers */
    interfaces->engineTrace->traceRay(Ray(src_3d - right * 35, dst_3d - right * 35), MASK_SHOT, { entity }, tr);
    float left_two = (tr.endpos - tr.startpos).length();
    /* fix side */
    if (left_two > right_two) {
        *side = -1;
    }
    else if (right_two > left_two) {
        *side = 1;
    }
    else
        *side = 0;
}

bool DesyncDetect(Entity* entity)
{
    if (!localPlayer)
        return false;
    if (!entity || !entity->isAlive())
        return false;
    if (entity->isBot())
        return false;
    if (entity->team() == localPlayer->team())
        return false;
    if (entity->moveType() == MoveType::NOCLIP || entity->moveType() == MoveType::LADDER)
        return false;
    return true;
}

bool isSlowWalking(Entity* entity)
{
    float velocity_2D[64]{}, old_velocity_2D[64]{};
    if (entity->velocity().length2D() != velocity_2D[entity->index()] && entity->velocity().length2D() != NULL) {
        old_velocity_2D[entity->index()] = velocity_2D[entity->index()];
        velocity_2D[entity->index()] = entity->velocity().length2D();
    }
    Vector velocity = entity->velocity();
    Vector direction = entity->eyeAngles();

    float speed = velocity.length();
    direction.y = entity->eyeAngles().y - direction.y;
    //method 1
    if (velocity_2D[entity->index()] > 1) {
        int tick_counter[64]{};
        if (velocity_2D[entity->index()] == old_velocity_2D[entity->index()])
            tick_counter[entity->index()] += 1;
        else
            tick_counter[entity->index()] = 0;

        while (tick_counter[entity->index()] > (1 / memory->globalVars->intervalPerTick * fabsf(0.1f)))//should give use 100ms in ticks if their speed stays the same for that long they are definetely up to something..
            return true;
    }


    return false;
}

int GetChokedPackets(Entity* entity) {
    int last_ticks[65]{};
    auto ticks = timeToTicks(entity->simulationTime() - entity->oldSimulationTime());
    if (ticks == 0 && last_ticks[entity->index()] > 0) {
        return last_ticks[entity->index()] - 1;
    }
    else {
        last_ticks[entity->index()] = ticks;
        return ticks;
    }
}

void Resolver::resolve_entity(const Animations::Players & player, Entity * entity) {
    // Check if desync is detected
    if (!DesyncDetect(entity))
        return;

    // Get the player's max rotation and eye yaw
    float max_rotation = entity->getMaxDesyncAngle();
    const float eye_yaw = entity->getAnimstate()->eyeYaw;

    // Adjust max rotation based on player's extended status
    // This is done to account for the difference in max rotation between extended and non-extended players
    if (!player.extended && fabs(max_rotation) > 60.f) {
        max_rotation = max_rotation / 1.8f;
    }

    // Resolve shooting players separately
    // Shooting players require a different resolution strategy, so they are handled separately
    if (player.shot) {
        entity->getAnimstate()->footYaw = eye_yaw + resolve_shot(player, entity);
        return;
    }

    int index = 0;
    // If player's velocity is low, calculate index based on angle difference
    // When the player's velocity is low, the angle difference between the eye yaw and foot yaw can be used to determine the player's direction
    if (entity->velocity().length2D() <= 2.5f) {
        const float angle_difference = Helpers::angleDiff(eye_yaw, entity->getAnimstate()->footYaw);
        index = (2 * angle_difference <= 0.0f) ? 1 : -1;
    }
    // If player's velocity is high, calculate index based on layer delta values
    // When the player's velocity is high, the delta values between the animation layers can be used to determine the player's direction
    else {
        if (!static_cast<int>(player.layers[12].weight * 1000.f) && entity->velocity().length2D() > 3.f) {
            float layer_delta1 = abs(player.layers[6].playbackRate - player.oldlayers[6].playbackRate);
            float layer_delta2 = abs(player.layers[7].playbackRate - player.oldlayers[7].playbackRate);
            float layer_delta3 = abs(player.layers[8].playbackRate - player.oldlayers[8].playbackRate);

            if (layer_delta1 < layer_delta2 || layer_delta3 <= layer_delta2 || static_cast<signed int>((layer_delta2 * 1000.0f))) {
                if (layer_delta1 >= layer_delta3 && layer_delta2 > layer_delta3 && !static_cast<signed int>((layer_delta3 * 1000.0f))) {
                    index = 1;
                }
            }
            else {
                index = -1;
            }
        }
    }

    // Calculate footYaw based on player's misses
    switch (player.misses % 3) {
    case 0: //default
        entity->getAnimstate()->footYaw = build_server_abs_yaw(entity, eye_yaw + max_rotation * static_cast<float>(index));
        break;
    case 1: //reverse
        entity->getAnimstate()->footYaw = build_server_abs_yaw(entity, eye_yaw + max_rotation * static_cast<float>(-index));
        break;
    case 2: //middle
        entity->getAnimstate()->footYaw = build_server_abs_yaw(entity, eye_yaw);
        break;
    case 3: //new mode 1: use the average of eye_yaw and footYaw 
        entity->getAnimstate()->footYaw = build_server_abs_yaw(entity, (eye_yaw + entity->getAnimstate()->footYaw) / 2.0f);
        break;
    case 4: //new mode 2: use the velocity direction to calculate footYaw
        Vector velocity = entity->velocity();
        if (velocity.length2D() > 0.1f) {
            entity->getAnimstate()->footYaw = build_server_abs_yaw(entity, Helpers::rad2deg(atan2(velocity.y, velocity.x)));
        }
    case 5: //new mode 3: use a fixed offset based on the player's index
        entity->getAnimstate()->footYaw = build_server_abs_yaw(entity, eye_yaw + 45.0f * static_cast<float>(index));
        break;
        default:
        // Handle unexpected values
        break;
    }
}

bool IsAdjustingBalance(const Animations::Players& player, Entity* entity)
{
    for (int i = 0; i < 15; i++)
    {
        const int activity = entity->sequence();
        if (activity == 979)
        {
            return true;
        }
    }
    return false;
}

bool is_breaking_lby(const Animations::Players& player, Entity* entity, AnimationLayer cur_layer, AnimationLayer prev_layer)
{
    if (IsAdjustingBalance(player, entity))
    {
        if ((prev_layer.cycle != cur_layer.cycle) || cur_layer.weight == 1.f)
        {
            return true;
        }
        else if (cur_layer.cycle == 0.f && (prev_layer.cycle > 0.92f && cur_layer.cycle > 0.92f))
        {
            return true;
        }
    }

    return false;
}

float Resolver::resolve_shot(const Animations::Players& player, Entity* entity) {
    /* fix unrestricted shot */
    if (DesyncDetect(entity) == false)
        return 0;

    const float fl_pseudo_fire_yaw = Helpers::angleNormalize(Helpers::angleDiff(localPlayer->origin().y, player.matrix[8].origin().y));
    if (is_breaking_lby(player, entity, player.layers[3], player.oldlayers[3])) {
        const float fl_left_fire_yaw_delta = fabsf(Helpers::angleNormalize(fl_pseudo_fire_yaw - (entity->eyeAngles().y + 58.f)));
        const float fl_right_fire_yaw_delta = fabsf(Helpers::angleNormalize(fl_pseudo_fire_yaw - (entity->eyeAngles().y - 58.f)));
        return fl_left_fire_yaw_delta > fl_right_fire_yaw_delta ? -58.f : 58.f;
    }
    const float fl_left_fire_yaw_delta = fabsf(Helpers::angleNormalize(fl_pseudo_fire_yaw - (entity->eyeAngles().y + 28.f)));
    const float fl_right_fire_yaw_delta = fabsf(Helpers::angleNormalize(fl_pseudo_fire_yaw - (entity->eyeAngles().y - 28.f)));

    return fl_left_fire_yaw_delta > fl_right_fire_yaw_delta ? -28.f : 28.f;
}

void Resolver::setup_detect(Animations::Players& player, Entity* entity) {

    // detect if player is using maximum desync.
    if (is_breaking_lby(player, entity, player.layers[3], player.oldlayers[3]))
    {
        player.extended = true;
    }
    /* calling detect side */
    detect_side(entity, &player.side);
    const int side = player.side;
    /* brute-forcing vars */
    float resolve_value = 50.f;
    static float brute = 0.f;
    const float fl_max_rotation = entity->getMaxDesyncAngle();
    const float fl_eye_yaw = entity->getAnimstate()->eyeYaw;
    const bool fl_forward = fabsf(Helpers::angleNormalize(get_angle(entity) - get_forward_yaw(entity))) < 90.f;
    const int fl_shots = player.misses;

    /* clamp angle */
    if (fl_max_rotation < resolve_value) {
        resolve_value = fl_max_rotation;
    }

    /* detect if entity is using max desync angle */
    if (player.extended) {
        resolve_value = fl_max_rotation;
    }

    const float perfect_resolve_yaw = resolve_value;

    /* setup brute-forcing */
    if (fl_shots == 0) {
        brute = perfect_resolve_yaw * static_cast<float>(fl_forward ? -side : side);
    }
    else {
        switch (fl_shots % 3) {
        case 0: {
            brute = perfect_resolve_yaw * static_cast<float>(fl_forward ? -side : side);
        } break;
        case 1: {
            brute = perfect_resolve_yaw * static_cast<float>(fl_forward ? side : -side);
        } break;
        case 2: {
            brute = 0;
        } break;
        default: break;
        }
    }

    /* fix goal feet yaw */
    entity->getAnimstate()->footYaw = fl_eye_yaw + brute;
}

// Calculate the angles required to aim from a source position to an entity position
Vector calculate_aim_angles(const Vector& source_position, const Vector& entity_position, const Vector& view_angles) {
    // Calculate the delta between the source and entity positions
    Vector delta = source_position - entity_position;

    // Extract the view angles
    const auto& [pitch, yaw, roll] = view_angles;

    // Calculate the pitch angle
    float pitch_angle = Helpers::rad2deg(std::atan2(delta.z, std::hypot(delta.x, delta.y))) - pitch;

    // Calculate the yaw angle
    float yaw_angle = Helpers::rad2deg(std::atan2(delta.y, delta.x)) - yaw;

    // Adjust the yaw angle if necessary
    if (delta.x >= 0.f) {
        yaw_angle += 180.f;
    }

    // Normalize the yaw angle to the range [-180, 180]
    yaw_angle = std::remainder(yaw_angle, 360.f);
    if (yaw_angle > 180.f) {
        yaw_angle -= 360.f;
    }
    else if (yaw_angle < -180.f) {
        yaw_angle += 360.f;
    }

    // Return the calculated angles
    return { pitch_angle, yaw_angle, 0.f };
}

void Resolver::processMissedShots() noexcept {
    // Check if resolver is enabled
    if (!resolver) {
        snapshots.clear();
        return;
    }

    // Check if local player is valid
    if (!localPlayer) {
        snapshots.clear();
        return;
    }

    // Check if snapshots are empty
    if (snapshots.empty()) {
        return;
    }

    // Check if snapshot data is not available yet
    if (snapshots.front().time == -1) {
        return;
    }

    // Get the snapshot data and remove it from the list
    auto snapshot = snapshots.front();
    snapshots.pop_front();

    // Calculate the current time
    const auto currentTime = localPlayer->isAlive() ? localPlayer->tickBase() * memory->globalVars->intervalPerTick : memory->globalVars->currenttime;

    // Check if the snapshot is too old
    if (std::abs(currentTime - snapshot.time) > 1.f) {
        if (snapshot.gotImpact) {
            Logger::addLog("Missed shot due to ping");
        } else {
            Logger::addLog("Missed shot due to server rejection");
        }
        snapshots.clear();
        return;
    }

    // Check if the snapshot has a valid matrix
    if (!snapshot.player.gotMatrix) {
        return;
    }

    // Get the entity and model data
    const auto entity = interfaces->entityList->getEntity(snapshot.playerIndex);
    if (!entity) {
        return;
    }

    const Model* model = snapshot.model;
    if (!model) {
        return;
    }

    StudioHdr* hdr = interfaces->modelInfo->getStudioModel(model);
    if (!hdr) {
        return;
    }

    StudioHitboxSet* set = hdr->getHitboxSet(0);
    if (!set) {
        return;
    }

    // Calculate the relative angle and end position for hitscan
    const auto angle = hitscan::calculateRelativeAngle(snapshot.eyePosition, snapshot.bulletImpact, Vector{});
    const auto endPosition = snapshot.bulletImpact + Vector::fromAngle(angle) * 2000.f;

    // Get the matrix data for hitscan
    const auto matrix = snapshot.backtrackRecord <= -1 ? snapshot.player.matrix.data() : snapshot.player.backtrackRecords.at(snapshot.backtrackRecord).matrix;

    // Check for missed shots due to resolver, backtrack, prediction error, or jitter
    bool resolverMissed = false;
    for (int hitbox = 0; hitbox < Hitboxes::Max; hitbox++) {
        if (hitscan::hitboxIntersection(matrix, hitbox, set, snapshot.eyePosition, endPosition)) {
            resolverMissed = true;
            std::string missedReason;
            if (snapshot.backtrackRecord == 1 && config->backtrack.enabled) {
                missedReason = "jitter";
            } else if (snapshot.backtrackRecord > 1 && config->backtrack.enabled) {
                missedReason = "invalid backtrack tick [" + std::to_string(snapshot.backtrackRecord) + "]";
            } else {
                missedReason = "resolver";
            }
            Logger::addLog("Missed shot on " + entity->getPlayerName() + " due to " + missedReason);
            Animations::setPlayer(snapshot.playerIndex)->misses++;
            break;
        }
    }

    // Log missed shots due to spread
    if (!resolverMissed) {
        Logger::addLog("Missed shot due to spread");
    }
}

void Resolver::runPreUpdate(Animations::Players player, Entity* entity) noexcept
{
    // Check essential conditions in one line for readability
    if (!resolver || !entity || !entity->isAlive() || entity->isDormant() || player.chokedPackets <= 0 || snapshots.empty()) {
        return;
    }

    // Access the snapshot data only after passing initial checks
    auto& [snapshot_player, model, eyePosition, bulletImpact, gotImpact, time, playerIndex, backtrackRecord] = snapshots.front();

    // Execute the main resolving functions
    setup_detect(player, entity);
    resolve_entity(player, entity);
}

void Resolver::runPostUpdate(Animations::Players player, Entity* entity) noexcept
{
    // Check essential conditions in one line for readability
    if (!resolver || !entity || !entity->isAlive() || entity->isDormant() || player.chokedPackets <= 0 || snapshots.empty()) {
        return;
    }

    // Execute the main resolving functions without unnecessary snapshot access
    setup_detect(player, entity);
    resolve_entity(player, entity);
}

void Resolver::updateEventListeners(bool forceRemove) noexcept
{
    class ImpactEventListener : public GameEventListener {
    public:
        void fireGameEvent(GameEvent* event) {
            getEvent(event);
        }
    };

    static ImpactEventListener listener[4];
    static const char* eventNames[] = {
        "bullet_impact",
        "player_hurt",
        "round_start",
        "weapon_fire"
    };

    static bool listenerRegistered = false;

    if (resolver && !listenerRegistered) {
        for (int i = 0; i < 4; ++i) {
            interfaces->gameEventManager->addListener(&listener[i], eventNames[i]);
        }
        listenerRegistered = true;
    }
    else if ((!resolver || forceRemove) && listenerRegistered) {
        for (int i = 0; i < 4; ++i) {
            interfaces->gameEventManager->removeListener(&listener[i]);
        }
        listenerRegistered = false;
    }
}