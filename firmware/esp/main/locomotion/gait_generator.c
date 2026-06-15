/**
 * @file gait_generator.c
 * @brief 2-DOF hexapod gait generation and servo coordination
 *
 * Coordinates 12 servos (6 legs x 2 DOF) on the body PCA9685 (0x40) for
 * walking and turning. Supports tripod, wave, and ripple gait patterns.
 * All I2C transactions use retry with bus recovery on timeout.
 */

#include "gait_generator.h"
#include "balance_control.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

#define NEUTRAL_STEP_DEG  4.0f

static const char *TAG = "GAIT";


/* GAIT DIAGRAM (top view)
*
*       [3]                    [12]
*          \                  /
*           [0]----FRONT---[15]
*            |              |
*      [4]--[1]----BODY----[14]--[11]
*            |              |
*           [2]----REAR----[13]
*          /                  \ 
*       [5]                    [10]
*/

static const uint8_t leg_channels[NUM_LEGS][DOF_PER_LEG] = {
    {15, 12},    // Leg 0: Front-Right  hip=ch15, knee=ch12
    {14, 11},    // Leg 1: Mid-Right    hip=ch14, knee=ch11
    {13, 10},    // Leg 2: Rear-Right   hip=ch13, knee=ch10
    {2,  5},    // Leg 3: Rear-Left    hip=ch2,  knee=ch5
    {1,  4},    // Leg 4: Mid-Left     hip=ch1,  knee=ch4
    {0,  3},    // Leg 5: Front-Left   hip=ch0,  knee=ch3
};

// Per-leg knee neutral positions. Set values in gait_generator.h.
static const float knee_neutral_deg[NUM_LEGS] = {
    KNEE_NEUTRAL_L0,  // Leg 0: Front-Right
    KNEE_NEUTRAL_L1,  // Leg 1: Mid-Right
    KNEE_NEUTRAL_L2,  // Leg 2: Rear-Right
    KNEE_NEUTRAL_L3,  // Leg 3: Rear-Left
    KNEE_NEUTRAL_L4,  // Leg 4: Mid-Left
    KNEE_NEUTRAL_L5,  // Leg 5: Front-Left
};

// Per-leg hip neutral positions. Set values in gait_generator.h.
static const float hip_neutral_deg[NUM_LEGS] = {
    HIP_NEUTRAL_L0,  // Leg 0: Front-Right
    HIP_NEUTRAL_L1,  // Leg 1: Mid-Right
    HIP_NEUTRAL_L2,  // Leg 2: Rear-Right
    HIP_NEUTRAL_L3,  // Leg 3: Rear-Left
    HIP_NEUTRAL_L4,  // Leg 4: Mid-Left
    HIP_NEUTRAL_L5,  // Leg 5: Front-Left
};

// gait configs
static const int8_t hip_direction[NUM_LEGS]  = {-1, -1, -1,  1,  1,  1};
static const int8_t knee_direction[NUM_LEGS] = { 1,  1,  1, -1, -1, -1};

static const float tripod_offsets[NUM_LEGS] = {
    0.0f, 0.5f, 0.0f, 0.5f, 0.0f, 0.5f,
};

static const float wave_offsets[NUM_LEGS] = {
    0.000f, 0.167f, 0.333f, 0.500f, 0.667f, 0.833f,
};

static const float ripple_offsets[NUM_LEGS] = {
    0.000f, 0.333f, 0.667f, 0.500f, 0.833f, 0.167f,
};

// walking configs
static const float gait_duty_cycle[] = {
    [GAIT_TRIPOD] = 0.5f,
    [GAIT_WAVE]   = 0.167f,
    [GAIT_RIPPLE] = 0.333f,
};

static gait_type_t current_gait = GAIT_TRIPOD;
static move_command_t current_command = MOVE_STOP;
static float current_speed = 0.5f;
static float master_phase = 0.0f;
static leg_angles_t current_angles = {0};
static const float *active_offsets = tripod_offsets;

static void compute_leg(float leg_phase, float duty, float stride,
                        float direction, int8_t hip_dir,
                        float *out_hip, float *out_knee) {

    // leg phase is the master_gait + leg_offsets of trig function

    // compute leg angles based on phase in step cycle
    // swing = foot in air, moves forward. stance = foot on ground, pushes body.
    // cosine easing: smooth_t goes 0→1 with soft acceleration/deceleration

    if (leg_phase < duty) { // if leg is in swing phase (0.0 - 0.5 in cycle)
        // get percentage of swing completed (0.0 - 1.0)
        float swing_progress = leg_phase / duty;

        // smooth it out with cosine mapping
        float smooth_t = 0.5f * (1.0f - cosf(swing_progress * M_PI));


        float hip_offset = (-stride / 2.0f) + (stride * smooth_t);
        *out_hip = HIP_NEUTRAL_DEG + (hip_offset * direction * hip_dir);

        // smooth knee lift: cosine ramp up 25%, hold 50%, cosine ramp down 25%
        float lift;
        if (swing_progress < 0.25f) { // lifting up
            float t = swing_progress / 0.25f; // take relative to the lift phase
            lift = 0.5f * (1.0f - cosf(t * M_PI)) * KNEE_LIFT_DEG; // map cosine to 0→1 and scale by max lift
        } else if (swing_progress < 0.75f) { // in the middle of the swing, hold max lift
            lift = KNEE_LIFT_DEG;
        } else { // moving down
            float t = (swing_progress - 0.75f) / 0.25f;
            lift = 0.5f * (1.0f + cosf(t * M_PI)) * KNEE_LIFT_DEG;
        }
        *out_knee = KNEE_NEUTRAL_DEG - lift;


    } else {
        // get progress from 0-1
        float stance_progress = (leg_phase - duty) / (1.0f - duty); 


        float smooth_t = 0.5f * (1.0f - cosf(stance_progress * M_PI));
        float hip_offset = (stride / 2.0f) - (stride * smooth_t);
        *out_hip = HIP_NEUTRAL_DEG + (hip_offset * direction * hip_dir);
        *out_knee = KNEE_NEUTRAL_DEG;
    }
}

esp_err_t xApplyAngles(const leg_angles_t *angles) {
    for (int i = 0; i < NUM_LEGS; i++) {
        uint8_t hip_angle  = (uint8_t)fmaxf(0, fminf(180, angles->hip_angle[i]));
        uint8_t knee_angle = (uint8_t)fmaxf(0, fminf(180, angles->knee_angle[i]));

        esp_err_t ret = xPCA9685SetAngleWithRetry(I2C_MASTER_NUM, PCA9685_BODY_ADDR,
                                                   leg_channels[i][0], hip_angle, SERVO_PWM_FREQ_HZ);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set leg %d hip after retries", i);
            return ret;
        }

        ret = xPCA9685SetAngleWithRetry(I2C_MASTER_NUM, PCA9685_BODY_ADDR,
                                         leg_channels[i][1], knee_angle, SERVO_PWM_FREQ_HZ);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set leg %d knee after retries", i);
            return ret;
        }
    }
    return ESP_OK;
}

esp_err_t xGaitInit(void) {
    // init gait system, put all legs in neutral position
    current_gait = GAIT_TRIPOD;
    current_command = MOVE_STOP;
    current_speed = 0.5f;
    master_phase = 0.0f;
    active_offsets = tripod_offsets;

    esp_err_t ret = xPCA9685Init(I2C_MASTER_NUM, PCA9685_BODY_ADDR, SERVO_PWM_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 body init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // set angles to neutral BEFORE first write to avoid servo jump from 0°
    for (int i = 0; i < NUM_LEGS; i++) {
        current_angles.hip_angle[i]  = hip_neutral_deg[i];
        current_angles.knee_angle[i] = knee_neutral_deg[i];
    }
    return xApplyAngles(&current_angles);
}

void vGaitSetCommand(move_command_t cmd, float speed) {
    // compute speed for legs based on command, with bounds checking on speed
    if (speed < 0.0f) speed = 0.0f;
    if (speed > 1.0f) speed = 1.0f;
    current_command = cmd;
    current_speed = speed;
}

void vGaitSetType(gait_type_t type) {
    if (current_gait == type) return;  // Already in this gait

    // change gait walking type, do NOT reset master_phase so legs finish their current step, doesn't teleport back to phase-0 position
    current_gait = type;
    switch (type) {
        case GAIT_TRIPOD: active_offsets = tripod_offsets;  break;
        case GAIT_WAVE:   active_offsets = wave_offsets;    break;
        case GAIT_RIPPLE: active_offsets = ripple_offsets;  break;
    }
    ESP_LOGI(TAG, "Gait changed to %d (phase=%.2f)", type, master_phase);
}

esp_err_t xGaitUpdate(const float *knee_corrections) {
    if (current_command == MOVE_STOP) {
        return ESP_OK;
    }

    // amount of updates needed to complete one full step cycle, based on speed. faster speed = faster phase increment.
    float effective_cycle_ms = STEP_CYCLE_MS / fmaxf(current_speed, 0.1f);
    float phase_increment = (float)GAIT_UPDATE_MS / effective_cycle_ms;

    master_phase += phase_increment;
    if (master_phase >= 1.0f) { master_phase -= 1.0f; }

    float duty = gait_duty_cycle[current_gait];
    float stride = HIP_STRIDE_DEG * current_speed;

    float leg_direction[NUM_LEGS];

    // states of walking: forward/backward/turning, determines leg movement directions. turning in place by having opposite directions on each side.
    switch (current_command) {
        case MOVE_FORWARD:
            for (int i = 0; i < NUM_LEGS; i++) leg_direction[i] = 1.0f;
            break;
        case MOVE_BACKWARD:
            for (int i = 0; i < NUM_LEGS; i++) leg_direction[i] = -1.0f;
            break;
        case MOVE_TURN_RIGHT:
            for (int i = 0; i < 3; i++) leg_direction[i] = -1.0f;
            for (int i = 3; i < 6; i++) leg_direction[i] =  1.0f;
            break;
        case MOVE_TURN_LEFT:
            for (int i = 0; i < 3; i++) leg_direction[i] =  1.0f;
            for (int i = 3; i < 6; i++) leg_direction[i] = -1.0f;
            break;
        default:
            return ESP_OK;
    }

    // send to all legs, compute target and distance from target based on phase in step cycle
    for (int i = 0; i < NUM_LEGS; i++) {
        float leg_phase = master_phase + active_offsets[i];
        if (leg_phase >= 1.0f) leg_phase -= 1.0f;

        // compute single leg angles based on gait phase and walking state
        compute_leg(leg_phase, duty, stride, leg_direction[i], hip_direction[i],
                    &current_angles.hip_angle[i], &current_angles.knee_angle[i]);

        // flip knee lift direction for left-side legs (servos are mirrored)
        float knee_dev = current_angles.knee_angle[i] - KNEE_NEUTRAL_DEG;
        current_angles.knee_angle[i] = KNEE_NEUTRAL_DEG + knee_dev * knee_direction[i];

        // shift to per-leg neutrals (compute_leg outputs relative to reference 90°)
        current_angles.knee_angle[i] += (knee_neutral_deg[i] - KNEE_NEUTRAL_DEG);
        current_angles.hip_angle[i]  += (hip_neutral_deg[i]  - HIP_NEUTRAL_DEG);

        if (knee_corrections != NULL) {
            current_angles.knee_angle[i] += knee_corrections[i] * knee_direction[i];
        }
    }

    // apply the angles to the servos
    return xApplyAngles(&current_angles);
}

esp_err_t xGaitStandNeutral(void) {
    ESP_LOGI(TAG, "Moving to neutral stance");

    // move all legs to neutral position gradually to avoid sudden jerks, with simple step interpolation
    bool still_moving = true;
    while (still_moving) {
        still_moving = false;
        for (int i = 0; i < NUM_LEGS; i++) {
            // graudally step towards neutral for all legs
            
            float hip_err  = hip_neutral_deg[i] - current_angles.hip_angle[i];
            float knee_err = knee_neutral_deg[i] - current_angles.knee_angle[i];

            if (fabsf(hip_err) > NEUTRAL_STEP_DEG) {
                current_angles.hip_angle[i]  += (hip_err  > 0.0f) ? NEUTRAL_STEP_DEG : -NEUTRAL_STEP_DEG;
                still_moving = true;
            } else {
                current_angles.hip_angle[i]  = hip_neutral_deg[i];
            }

            if (fabsf(knee_err) > NEUTRAL_STEP_DEG) {
                current_angles.knee_angle[i] += (knee_err > 0.0f) ? NEUTRAL_STEP_DEG : -NEUTRAL_STEP_DEG;
                still_moving = true;
            } else {
                current_angles.knee_angle[i] = knee_neutral_deg[i];
            }
        }

        esp_err_t ret = xApplyAngles(&current_angles);
        if (ret != ESP_OK) return ret;

        if (still_moving) vTaskDelay(pdMS_TO_TICKS(GAIT_UPDATE_MS));
    }

    return ESP_OK;
}

leg_angles_t xGaitGetAngles(void) {
    return current_angles;
}

bool bGaitStepComplete(void) {
    // True when master_phase is near the start of a new cycle — all legs are
    // close to their neutral stance positions, so a gait transition or stop
    // won't cause a mid-stride snap back to neutral.
    return (master_phase < 0.1f || master_phase > 0.9f);
}