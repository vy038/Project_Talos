#include "state_machine.h"
#include "gait_generator.h"
#include "balance_control.h"
#include "ik_solver.h"
#include "arm_control.h"
#include "power_management.h"
#include "mpu6050.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "STATE";

static robot_state_t current_state = STATE_INIT;
static int64_t state_enter_time = 0;
static detection_result_t last_detection = {0};
static bool grab_prep_arm_sent = false;
static uint16_t s_prev_tof_mm = 0;            // last valid TOF reading during approach, for spike detection

// IK result saved during GRAB_PREP so it can be printed after the grab completes TODO: REMOVE AFTER TROUBLESHOOTING
static struct {
    float base, shoulder, elbow;
    float tof_mm;
    bool  ik_used;      // true = IK angles, false = scoop fallback
    bool  populated;
} s_last_ik_result = {0};


// get time spent in state
static uint32_t ms_in_state(void) {
    return (uint32_t)((esp_timer_get_time() - state_enter_time) / 1000);
}

static void transition(robot_state_t new_state) {
    // transition to a new state and reset timer
    ESP_LOGI(TAG, "Transition: %d -> %d (was in state for %lums)",
             current_state, new_state, (unsigned long)ms_in_state());
    if (new_state != STATE_GRAB_PREP) {
        grab_prep_arm_sent = false;
    }
    s_prev_tof_mm = 0;
    current_state = new_state;
    state_enter_time = esp_timer_get_time();
}

// TODO: calibrate these by manually posing the arm and recording angles
static const arm_angles_t arm_grab_ready = {
    .base     = 90.0f,
    .shoulder = 55.0f,
    .elbow    = 65.0f,
    .gripper  = 170.0f,
};

static const arm_angles_t arm_lifted = {
    .base     = 90.0f,
    .shoulder = 120.0f,
    .elbow    = 90.0f,
    .gripper  = 0.0f,
};

static void handle_init(void) {
    // starting state
    transition(STATE_CALIBRATE);
}

static void handle_calibrate(void) {
    // calibrate everything and move to idle, requires robot to be still for a few seconds (assume everything is initialized)
    if (ms_in_state() < 100) {
        ESP_LOGI(TAG, "Starting MPU6050 calibration - keep robot STILL");
        xMPU6050_calibrate();
        xBalanceInit();
        return;
    }

    if (ms_in_state() > 2000) {
        ESP_LOGI(TAG, "Calibration complete");
        transition(STATE_IDLE);
    }
}

static void handle_idle(void) {
    // transition from idle to moving stop state, wait for a moment to stabilize after calibration
    vGaitSetCommand(MOVE_STOP, 0);
    if (ms_in_state() > 2000) {
        transition(STATE_SEARCH);
    }
}

static void handle_search(void) {
    vGaitSetType(GAIT_TRIPOD);
    // tells robot to turn 360 right slowly while looking for the ball
    vGaitSetCommand(MOVE_TURN_RIGHT, 0.2f);

    // stops when ball is found
    if (last_detection.fresh && last_detection.detected) {
        last_detection.fresh = false;
        ESP_LOGI(TAG, "Ball detected at x=%d, px_r=%d",
                 last_detection.ball_x, last_detection.pixel_radius);

        int center_offset = (int)last_detection.ball_x - (CAM_FRAME_WIDTH / 2);

        if (abs(center_offset) < BALL_CENTER_TOLERANCE_X) {
            transition(STATE_APPROACH);
        } else {
            transition(STATE_ALIGN);
        }
        return;
    }

    // keeps looking for a while before giving up and trying again
    if (ms_in_state() > SEARCH_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Search timeout, continuing to search...");
        state_enter_time = esp_timer_get_time();
    }
}

static void handle_align(void) {
    // use tripod for coarse correction (avoids mid-stride gait switch from searching, causing janky leg movement)
    int center_offset = (int)last_detection.ball_x - (CAM_FRAME_WIDTH / 2);
    if (abs(center_offset) > ALIGN_COARSE_THRESHOLD_X) {
        vGaitSetType(GAIT_TRIPOD);
    } else { // switch to wave only for fine alignment when nearly centred
        vGaitSetType(GAIT_WAVE);
    }

    if (!last_detection.fresh) {
        if (ms_in_state() > 2000) {
            ESP_LOGW(TAG, "Lost ball during alignment");
            transition(STATE_SEARCH);
        }
        return;
    }

    last_detection.fresh = false;

    if (!last_detection.detected) {
        transition(STATE_SEARCH);
        return;
    }

    if (abs(center_offset) < BALL_CENTER_TOLERANCE_X) {
        vGaitSetCommand(MOVE_STOP, 0);
        transition(STATE_APPROACH);
        return;
    }

    // scale turn speed from pixel radius: bigger radius = ball is close = turn slower for precision
    // camera distance estimate is noisy; pixel radius is stable enough for speed modulation only
    float px_ratio = (float)last_detection.pixel_radius / (float)MAX_TURN_PIXEL_RADIUS;
    if (px_ratio > 1.0f) px_ratio = 1.0f;
    float turn_speed = 0.2f - (0.12f * px_ratio);  // 0.2 when far/small, 0.08 when close/large
    if (turn_speed < 0.08f) turn_speed = 0.08f;

    if (center_offset > 0) {
        vGaitSetCommand(MOVE_TURN_RIGHT, turn_speed);
    } else {
        vGaitSetCommand(MOVE_TURN_LEFT, turn_speed);
    }
}

static void handle_approach(void) {
    vGaitSetType(GAIT_TRIPOD);

    // speed control: use VL53L0X as authoritative distance.
    // if TOF has no reading yet (ball not in beam), creep forward slowly.
    float walk_speed = APPROACH_BLIND_SPEED;
    if (last_detection.tof_dist_mm > 0 && last_detection.tof_dist_mm < BALL_APPROACH_FAR_MM) {
        float proximity = 1.0f - (float)last_detection.tof_dist_mm / (float)BALL_APPROACH_FAR_MM;
        walk_speed = 0.5f - (0.2f * proximity);   // 0.5 far, 0.3 close
        if (walk_speed < 0.2f) walk_speed = 0.2f; // safeguard
    }

    vGaitSetCommand(MOVE_FORWARD, walk_speed);

    if (!last_detection.fresh) {
        if (ms_in_state() > 3000) {
            ESP_LOGW(TAG, "Lost ball during approach");
            vGaitSetCommand(MOVE_STOP, 0);
            transition(STATE_SEARCH);
        }
        return;
    }

    last_detection.fresh = false;

    if (!last_detection.detected) {
        vGaitSetCommand(MOVE_STOP, 0);
        transition(STATE_SEARCH);
        return;
    }

    // camera centering check: if ball drifts significantly off-center, realign
    int center_offset = (int)last_detection.ball_x - (CAM_FRAME_WIDTH / 2);
    if (abs(center_offset) > BALL_CENTER_TOLERANCE_X * 2) {
        transition(STATE_ALIGN);
        return;
    }

    uint16_t tof = last_detection.tof_dist_mm;

    // TOF spike: ball left the beam (robot turned or ball moved). use camera to micro-adjust.
    if (tof > 0 && s_prev_tof_mm > 0 && (float)tof > (float)s_prev_tof_mm * TOF_SPIKE_RATIO) {
        ESP_LOGW(TAG, "TOF spike %d->%d mm, realigning", s_prev_tof_mm, tof);
        transition(STATE_ALIGN);
        return;
    }

    if (tof > 0) {
        s_prev_tof_mm = tof;
    }

    // stop and grab when VL53L0X confirms close enough
    if (tof > 0 && tof <= BALL_STOP_TOF_MM) {
        ESP_LOGI(TAG, "Ball within reach (tof=%d mm)", tof);
        vGaitSetCommand(MOVE_STOP, 0);
        transition(STATE_GRAB_PREP);
    }
}

static void handle_grab_prep(void) {
    // pause a bit before grabbing to allow robot to stabilize
    if (ms_in_state() < 200) {
        vGaitSetCommand(MOVE_STOP, 0);
        return;
    }

    // ensure its pausing before moving arm
    if (ms_in_state() < GRAB_PREP_PAUSE_MS) {
        return;
    }

    // move arm to proper grab position (after stabilized)
    if (!grab_prep_arm_sent) {
        // TOF reports front-surface distance, add ball radius to get the center
        float dist_mm = ((last_detection.tof_dist_mm > 0)
                        ? (float)last_detection.tof_dist_mm
                        : (float)BALL_STOP_TOF_MM)
                        + BALL_RADIUS_MM;

        // pixel offsets to angles: ball_x right = negative y (y is left)
        float px_h = (float)last_detection.ball_x - (CAM_FRAME_WIDTH  / 2.0f);
        float px_v = (CAM_FRAME_HEIGHT / 2.0f)    - (float)last_detection.ball_y;
        float ah   = px_h / (float)CAM_FRAME_WIDTH  * CAM_HFOV_DEG * (float)(M_PI / 180.0);
        float av   = px_v / (float)CAM_FRAME_HEIGHT * CAM_VFOV_DEG * (float)(M_PI / 180.0);

        // reconstruct 3D ball position in camera frame (x=forward, y=left, z=up)
        float r_h = dist_mm * cosf(av);
        ik_target_t cam_pos = {
            .x =  r_h * cosf(ah),
            .y = -r_h * sinf(ah),
            .z =  dist_mm * sinf(av),
        };

        ik_target_t arm_pos;
        xIKCameraToArm(&cam_pos, &arm_pos);

        ik_solution_t sol;
        arm_angles_t grab = arm_grab_ready;

        if (xIKSolve(&arm_pos, &sol) == ESP_OK && sol.valid) {
            // Shoulder clamping detection: if the raw IK angle was negative (target
            // genuinely below the arm's reach) the solver clamps it near zero.
            // Values < 10° indicate the IK was computing a negative angle, indicating
            // the arm geometry doesn't match that target. Use the calibrated scoop pose
            // instead, but keep IK's base rotation which is always valid.
            bool shoulder_floored = (sol.shoulder < 10.0f);

            grab.base = sol.base_rotation;
            if (!shoulder_floored) {
                grab.shoulder = sol.shoulder;
                grab.elbow    = sol.elbow;
            }
            // grab.shoulder/elbow/gripper stay at arm_grab_ready when floored

            s_last_ik_result.base     = grab.base;
            s_last_ik_result.shoulder = grab.shoulder;
            s_last_ik_result.elbow    = grab.elbow;
            s_last_ik_result.tof_mm   = dist_mm;
            s_last_ik_result.ik_used  = !shoulder_floored;
            s_last_ik_result.populated = true;
        } else {
            // IK unreachable, fall back to calibrated scoop pose with pixel-offset base
            float center_offset = (float)last_detection.ball_x - (CAM_FRAME_WIDTH / 2.0f);
            float angle_offset  = center_offset * (CAM_HFOV_DEG / (float)CAM_FRAME_WIDTH) * BASE_ANGLE_SCALE;
            grab.base = 90.0f - angle_offset;
            if (grab.base < BASE_ROTATION_MIN) grab.base = BASE_ROTATION_MIN;
            if (grab.base > BASE_ROTATION_MAX) grab.base = BASE_ROTATION_MAX;

            s_last_ik_result.base      = grab.base;
            s_last_ik_result.shoulder  = grab.shoulder;
            s_last_ik_result.elbow     = grab.elbow;
            s_last_ik_result.tof_mm    = dist_mm;
            s_last_ik_result.ik_used   = false;
            s_last_ik_result.populated = true;
        }

        xArmSetAngles(&grab);
        grab_prep_arm_sent = true;
    }

    // grab object
    if (bArmAtTarget()) {
        transition(STATE_GRAB);
    }

    if (ms_in_state() > GRAB_PREP_PAUSE_MS + 8000) {
        ESP_LOGE(TAG, "Arm move timeout");
        transition(STATE_IDLE);
    }
}

static void handle_grab(void) {
    // close gripper immediately on entry
    if (ms_in_state() < 50) {
        xArmGripper(0.0f);
        return;
    }
    // wait until gripper reaches target, transition once closed or after timeout
    // (ARM_STEP_DEG = 1/20ms = 50 deg/sec, 170 deg travel takes ~3.4s)
    if (bArmAtTarget()) {
        if (s_last_ik_result.populated) {
            if (s_last_ik_result.ik_used) {
                ESP_LOGI(TAG, "GRAB OK: IK solution: base=%.1f shoulder=%.1f elbow=%.1f (tof=%.0fmm)",
                         s_last_ik_result.base, s_last_ik_result.shoulder,
                         s_last_ik_result.elbow, s_last_ik_result.tof_mm);
            } else {
                ESP_LOGI(TAG, "GRAB OK: scoop fallback: base=%.1f shoulder=%.1f elbow=%.1f (tof=%.0fmm) "
                              "[IK shoulder was at safety limit]",
                         s_last_ik_result.base, s_last_ik_result.shoulder,
                         s_last_ik_result.elbow, s_last_ik_result.tof_mm);
            }
        }
        transition(STATE_LIFT);
        return;
    }
    if (ms_in_state() > 1500) {
        ESP_LOGW(TAG, "Gripper close timeout");
        transition(STATE_LIFT);
    }
}

static void handle_lift(void) {
    // lifting action for arm once object is grabbed
    if (ms_in_state() < 100) {
        ESP_LOGI(TAG, "Lifting object");
        xArmSetAngles(&arm_lifted);
        return;
    }

    // transition to done if done 
    if (bArmAtTarget()) {
        transition(STATE_DONE);
    }

    // transition to timeout if takes too long
    if (ms_in_state() > 6000) {
        ESP_LOGW(TAG, "Lift timeout, considering done anyway");
        transition(STATE_DONE);
    }
}

static void handle_done(void) {
    // final state if successfully grabbed object, just stop and celebrate
    vGaitSetCommand(MOVE_STOP, 0);
    if (ms_in_state() < 100) {
        ESP_LOGI(TAG, "Mission complete. Object acquired.");
    }
}

static void handle_emergency(void) {
    // battery overheat or robot tipping over, stop all motion and wait for manual reset
    if (ms_in_state() < 100) {
        ESP_LOGE(TAG, "EMERGENCY STATE - all motion stopped");
        vGaitSetCommand(MOVE_STOP, 0);
        vBalanceEnable(false);
        xGaitStandNeutral();
    }
}

esp_err_t xStateMachineInit(void) {
    // reset state machine and register power callback
    current_state = STATE_INIT;
    state_enter_time = esp_timer_get_time();
    memset(&last_detection, 0, sizeof(last_detection));
    ESP_LOGI(TAG, "State machine initialized");
    return ESP_OK;
}

esp_err_t xStateMachineUpdate(void) {
    // update state machine, should be called in main loop, handles transitions and state actions

    // checking for tipping over
    if (current_state != STATE_EMERGENCY && current_state != STATE_INIT) {
        if (bBalanceIsTipping(45.0f)) {
            ESP_LOGE(TAG, "Robot tipping! Entering emergency state");
            transition(STATE_EMERGENCY);
        }
    }

    // states to state handler
    switch (current_state) {
        case STATE_INIT:        handle_init();      break;
        case STATE_CALIBRATE:   handle_calibrate(); break;
        case STATE_IDLE:        handle_idle();      break;
        case STATE_SEARCH:      handle_search();    break;
        case STATE_ALIGN:       handle_align();     break;
        case STATE_APPROACH:    handle_approach();  break;
        case STATE_GRAB_PREP:   handle_grab_prep(); break;
        case STATE_GRAB:        handle_grab();      break;
        case STATE_LIFT:        handle_lift();      break;
        case STATE_DONE:        handle_done();      break;
        case STATE_EMERGENCY:   handle_emergency(); break;
    }

    return ESP_OK;
}

robot_state_t xStateMachineGetState(void) {
    return current_state;
}

void vStateMachineForceState(robot_state_t new_state) {
    transition(new_state);
}

void vStateMachineFeedDetection(detection_result_t result) {
    result.fresh = true;
    last_detection = result;
}

bool bStateMachineParseUART(const uint8_t *buf, size_t len, detection_result_t *result) {
    // uart handler for parsing ball detection messages from the camera module

    if (len < UART_MSG_LENGTH) return false;

    // validation
    for (size_t i = 0; i <= len - UART_MSG_LENGTH; i++) {
        if (buf[i] != UART_MSG_START_0 || buf[i + 1] != UART_MSG_START_1) {
            continue;
        }

        const uint8_t *msg = &buf[i];

        if (msg[2] != UART_MSG_TYPE_DETECT) {
            continue;
        }

        uint8_t checksum = 0;
        for (int j = 2; j < UART_MSG_LENGTH - 1; j++) {
            checksum ^= msg[j];
        }

        if (checksum != msg[UART_MSG_LENGTH - 1]) {
            ESP_LOGW(TAG, "UART checksum mismatch: got 0x%02X, expected 0x%02X",
                     msg[UART_MSG_LENGTH - 1], checksum);
            continue;
        }

        // parsing message into detection result struct
        result->detected     = (msg[3] != 0);
        result->ball_x       = (uint16_t)(msg[4]  << 8) | msg[5];
        result->ball_y       = (uint16_t)(msg[6]  << 8) | msg[7];
        result->pixel_radius = (uint16_t)(msg[8]  << 8) | msg[9];
        result->tof_dist_mm  = (uint16_t)(msg[10] << 8) | msg[11];
        result->fresh        = true;

        return true;
    }

    return false;
}