// gait_generator.c
#include "gait_generator.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "GAIT";

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0f)
#define RAD_TO_DEG(rad) ((rad) * 180.0f / M_PI)

// Leg attachment positions [x, y] in mm
static const float leg_attachments[NUM_LEGS][2] = {
    {LEG_FL_ATTACH_X, LEG_FL_ATTACH_Y},  // Front left
    {LEG_ML_ATTACH_X, LEG_ML_ATTACH_Y},  // Middle left
    {LEG_RL_ATTACH_X, LEG_RL_ATTACH_Y},  // Rear left
    {LEG_FR_ATTACH_X, LEG_FR_ATTACH_Y},  // Front right
    {LEG_MR_ATTACH_X, LEG_MR_ATTACH_Y},  // Middle right
    {LEG_RR_ATTACH_X, LEG_RR_ATTACH_Y}   // Rear right
};

// Phase offsets for each gait pattern
// Phase 0.0 = start of stance, 0.5 = start of swing for that leg
static const float gait_phase_offsets[4][NUM_LEGS] = {
    // Tripod gait: two groups of 3 legs
    {0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f},  // FL,RL,MR together; ML,FR,RR together
    
    // Wave gait: sequential, one leg at a time
    {0.0f, 0.167f, 0.333f, 0.5f, 0.667f, 0.833f},
    
    // Ripple gait: alternating groups
    {0.0f, 0.25f, 0.5f, 0.125f, 0.375f, 0.625f},
    
    // Stationary: all legs in stance
    {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}
};

// Duty cycle: fraction of gait cycle with leg on ground
static const float gait_duty_cycles[] = {
    0.5f,  // Tripod: 50% stance
    0.833f,  // Wave: 83.3% stance
    0.667f,  // Ripple: 66.7% stance
    1.0f   // Stationary: 100% stance
};

static gait_type_t current_gait = GAIT_TRIPOD;
static float global_phase = 0.0f;

esp_err_t xGaitGeneratorInit(void) {
    global_phase = 0.0f;
    current_gait = GAIT_TRIPOD;
    ESP_LOGI(TAG, "Gait generator initialized");
    return ESP_OK;
}

esp_err_t xGaitSetType(gait_type_t type) {
    if (type >= 4) {
        return ESP_ERR_INVALID_ARG;
    }
    current_gait = type;
    ESP_LOGI(TAG, "Gait type set to %d", type);
    return ESP_OK;
}

esp_err_t xGaitGetNeutralStance(leg_index_t leg_idx, leg_position_t *position) {
    if (!position || leg_idx >= NUM_LEGS) {
        return ESP_ERR_INVALID_ARG;
    }

    // Neutral stance: legs angled outward at 45 degrees from body
    float angle = atan2f(leg_attachments[leg_idx][1], leg_attachments[leg_idx][0]);
    
    // Distance from attachment point to foot in neutral stance
    float stance_radius = sqrtf(LEG_FEMUR_LENGTH * LEG_FEMUR_LENGTH + 
                               LEG_TIBIA_LENGTH * LEG_TIBIA_LENGTH);
    
    position->x = LEG_COXA_LENGTH + stance_radius * 0.7f * cosf(angle);  // 0.7 factor for angled stance
    position->y = stance_radius * 0.7f * sinf(angle);
    position->z = -DEFAULT_STANCE_HEIGHT;  // Negative because below body
    
    return ESP_OK;
}

// Calculate foot position during swing phase (lifted leg)
static void vCalculateSwingPosition(float leg_phase, float duty_cycle,
                                    const leg_position_t *start_pos,
                                    const leg_position_t *end_pos,
                                    float step_height,
                                    leg_position_t *current_pos) {
    // Swing phase: duty_cycle to 1.0
    float swing_progress = (leg_phase - duty_cycle) / (1.0f - duty_cycle);
    
    // Linear interpolation for X and Y
    current_pos->x = start_pos->x + swing_progress * (end_pos->x - start_pos->x);
    current_pos->y = start_pos->y + swing_progress * (end_pos->y - start_pos->y);
    
    // Parabolic arc for Z (smooth lift and lower)
    float height_progress = 4.0f * swing_progress * (1.0f - swing_progress);  // Peaks at 0.5
    current_pos->z = start_pos->z + height_progress * step_height;
}

// Calculate foot position during stance phase (on ground)
static void vCalculateStancePosition(float leg_phase, float duty_cycle,
                                     const leg_position_t *start_pos,
                                     const leg_position_t *end_pos,
                                     leg_position_t *current_pos) {
    // Stance phase: 0.0 to duty_cycle
    float stance_progress = leg_phase / duty_cycle;
    
    // Linear interpolation (foot slides backward relative to body moving forward)
    current_pos->x = start_pos->x + stance_progress * (end_pos->x - start_pos->x);
    current_pos->y = start_pos->y + stance_progress * (end_pos->y - start_pos->y);
    current_pos->z = start_pos->z;  // Stays at ground level
}

esp_err_t xGaitUpdate(const velocity_command_t *velocity, uint32_t dt_ms, gait_state_t *state) {
    if (!velocity || !state) {
        return ESP_ERR_INVALID_ARG;
    }

    // Update global phase based on time
    float phase_increment = (float)dt_ms / (float)GAIT_CYCLE_TIME_MS;
    global_phase += phase_increment;
    if (global_phase >= 1.0f) {
        global_phase -= 1.0f;
    }

    state->global_phase = global_phase;
    state->type = current_gait;
    state->step_height = DEFAULT_STEP_HEIGHT;
    state->stance_height = DEFAULT_STANCE_HEIGHT;

    float duty_cycle = gait_duty_cycles[current_gait];

    // Calculate displacement per gait cycle
    float cycle_time_s = (float)GAIT_CYCLE_TIME_MS / 1000.0f;
    float forward_displacement = velocity->forward_velocity * cycle_time_s;
    float lateral_displacement = velocity->lateral_velocity * cycle_time_s;
    float rotation_displacement = velocity->rotation_velocity * cycle_time_s;

    // Update each leg
    for (int i = 0; i < NUM_LEGS; i++) {
        // Calculate this leg's phase in its gait cycle
        float leg_phase = global_phase + gait_phase_offsets[current_gait][i];
        if (leg_phase >= 1.0f) {
            leg_phase -= 1.0f;
        }
        state->legs[i].phase = leg_phase;

        // Determine if leg is in swing or stance
        state->legs[i].in_swing = (leg_phase >= duty_cycle);

        // Get neutral stance position for this leg
        leg_position_t neutral_pos;
        xGaitGetNeutralStance(i, &neutral_pos);

        // Calculate body motion contribution to leg position
        // As body moves forward, feet move backward relative to body
        float body_rotation_rad = DEG_TO_RAD(rotation_displacement);
        float leg_attach_radius = sqrtf(leg_attachments[i][0] * leg_attachments[i][0] + 
                                       leg_attachments[i][1] * leg_attachments[i][1]);
        
        // Start and end positions for this leg's stride
        leg_position_t stride_start, stride_end;
        
        // Stride start: half stride behind neutral
        stride_start.x = neutral_pos.x + forward_displacement * 0.5f;
        stride_start.y = neutral_pos.y + lateral_displacement * 0.5f;
        stride_start.z = neutral_pos.z;
        
        // Stride end: half stride ahead of neutral
        stride_end.x = neutral_pos.x - forward_displacement * 0.5f;
        stride_end.y = neutral_pos.y - lateral_displacement * 0.5f;
        stride_end.z = neutral_pos.z;

        // Calculate current foot position based on phase
        if (state->legs[i].in_swing) {
            vCalculateSwingPosition(leg_phase, duty_cycle, 
                                   &stride_end, &stride_start,
                                   state->step_height,
                                   &state->legs[i].position);
        } else {
            vCalculateStancePosition(leg_phase, duty_cycle,
                                    &stride_start, &stride_end,
                                    &state->legs[i].position);
        }

        // Solve IK for this leg position
        esp_err_t err = xGaitLegIK(i, &state->legs[i].position, &state->legs[i].angles);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "IK failed for leg %d", i);
        }
    }

    return ESP_OK;
}

esp_err_t xGaitLegIK(leg_index_t leg_idx, const leg_position_t *target, leg_angles_t *angles) {
    if (!target || !angles || leg_idx >= NUM_LEGS) {
        return ESP_ERR_INVALID_ARG;
    }

    // Step 1: Solve coxa angle (horizontal rotation)
    // Coxa rotates to point toward target in XY plane
    angles->coxa = RAD_TO_DEG(atan2f(target->y, target->x));

    // Step 2: Calculate distance from coxa joint to target
    float horizontal_dist = sqrtf(target->x * target->x + target->y * target->y) - LEG_COXA_LENGTH;
    float vertical_dist = -target->z;  // Negative because Z is down
    float target_dist = sqrtf(horizontal_dist * horizontal_dist + vertical_dist * vertical_dist);

    // Check reachability
    float max_reach = LEG_FEMUR_LENGTH + LEG_TIBIA_LENGTH;
    float min_reach = fabsf(LEG_FEMUR_LENGTH - LEG_TIBIA_LENGTH);
    
    if (target_dist > max_reach || target_dist < min_reach) {
        ESP_LOGW(TAG, "Leg %d target unreachable: dist=%.2f, range=[%.2f, %.2f]",
                 leg_idx, target_dist, min_reach, max_reach);
        return ESP_ERR_INVALID_ARG;
    }

    // Step 3: Solve femur and tibia using 2D IK (law of cosines)
    // Calculate tibia angle
    float cos_tibia = (LEG_FEMUR_LENGTH * LEG_FEMUR_LENGTH + 
                       LEG_TIBIA_LENGTH * LEG_TIBIA_LENGTH - 
                       target_dist * target_dist) / 
                      (2.0f * LEG_FEMUR_LENGTH * LEG_TIBIA_LENGTH);
    cos_tibia = fmaxf(-1.0f, fminf(1.0f, cos_tibia));  // Clamp for numerical stability
    
    angles->tibia = RAD_TO_DEG(acosf(cos_tibia));

    // Calculate femur angle
    float angle_to_target = atan2f(vertical_dist, horizontal_dist);
    float cos_femur_offset = (LEG_FEMUR_LENGTH * LEG_FEMUR_LENGTH + 
                              target_dist * target_dist - 
                              LEG_TIBIA_LENGTH * LEG_TIBIA_LENGTH) / 
                             (2.0f * LEG_FEMUR_LENGTH * target_dist);
    cos_femur_offset = fmaxf(-1.0f, fminf(1.0f, cos_femur_offset));
    float femur_offset = acosf(cos_femur_offset);
    
    angles->femur = RAD_TO_DEG(angle_to_target + femur_offset);

    return ESP_OK;
}