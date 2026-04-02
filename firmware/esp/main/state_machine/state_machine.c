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

static const char *TAG = "STATE";

static robot_state_t current_state = STATE_INIT;
static int64_t state_enter_time = 0;
static detection_result_t last_detection = {0};
static bool grab_prep_arm_sent = false;
static gait_type_t active_gait = GAIT_TRIPOD; // initial gait

// helper function to switch gait if not already set, avoids mid-stride gait switches which cause leg snapping
static void set_gait_if_needed(gait_type_t type) {
    if (active_gait != type) {
        vGaitSetType(type);
        active_gait = type;
    }
}

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
    set_gait_if_needed(GAIT_TRIPOD);
    // tells robot to turn 360 right slowly while looking for the ball
    vGaitSetCommand(MOVE_TURN_RIGHT, 0.2f);

    // stops when ball is found
    if (last_detection.fresh && last_detection.detected) {
        last_detection.fresh = false;
        ESP_LOGI(TAG, "Ball detected at x=%d, r=%d",
                 last_detection.ball_x, last_detection.ball_radius);

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
    // use tripod for coarse correction (avoids mid-stride gait switch from searching, causing janky leg movement),
    int coarse_offset = (int)last_detection.ball_x - (CAM_FRAME_WIDTH / 2);
    if (abs(coarse_offset) > ALIGN_COARSE_THRESHOLD_X) {
        set_gait_if_needed(GAIT_TRIPOD);
    } else { // switch to wave only for fine alignment when nearly centred
        set_gait_if_needed(GAIT_WAVE);
    }

    // if ball is not updated since last read, look for ball TODO change method?
    if (!last_detection.fresh) {
        if (ms_in_state() > 2000) {
            ESP_LOGW(TAG, "Lost ball during alignment");
            transition(STATE_SEARCH);
        }
        return;
    }

    last_detection.fresh = false;

    // if ball lost, go back to search
    if (!last_detection.detected) {
        transition(STATE_SEARCH);
        return;
    }

    // calculate direction of ball rel to center
    int center_offset = (int)last_detection.ball_x - (CAM_FRAME_WIDTH / 2);

    if (abs(center_offset) < BALL_CENTER_TOLERANCE_X) {
        vGaitSetCommand(MOVE_STOP, 0);
        transition(STATE_APPROACH);
        return;
    }

    // scale turn speed down as ball gets closer (smaller dist_mm = closer)
    float proximity = 1.0f - (float)last_detection.ball_radius / (float)BALL_APPROACH_FAR_MM;
    if (proximity < 0.0f) proximity = 0.0f;
    if (proximity > 1.0f) proximity = 1.0f;
    float turn_speed = 0.2f - (0.12f * proximity);  // 0.2 when far, 0.08 when close
    if (turn_speed < 0.08f) turn_speed = 0.08f;

    // direction to move in
    if (center_offset > 0) {
        vGaitSetCommand(MOVE_TURN_RIGHT, turn_speed);
    } else {
        vGaitSetCommand(MOVE_TURN_LEFT, turn_speed);
    }
}

static void handle_approach(void) {
    set_gait_if_needed(GAIT_TRIPOD);
    // Use a moderate constant speed. ToF only fires when beam intersects ball,
    // so it can't be used as a general proximity proxy during the whole approach.
    float walk_speed = 0.3f;
    if (last_detection.ball_radius > 0 && last_detection.ball_radius < BALL_APPROACH_FAR_MM) {
        // ToF beam is actively hitting the ball — scale down as it gets closer
        float proximity = 1.0f - (float)last_detection.ball_radius / (float)BALL_APPROACH_FAR_MM;
        walk_speed = 0.3f - (0.15f * proximity);  // 0.3 → 0.15 as ball enters beam range
        if (walk_speed < 0.1f) walk_speed = 0.1f;
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

    // if not in camera frame anymore, look for it
    if (!last_detection.detected) {
        vGaitSetCommand(MOVE_STOP, 0);
        transition(STATE_SEARCH);
        return;
    }

    // if not in center, prepare to turn
    int center_offset = (int)last_detection.ball_x - (CAM_FRAME_WIDTH / 2);
    if (abs(center_offset) > BALL_CENTER_TOLERANCE_X * 2) {
        transition(STATE_ALIGN);
        return;
    }

    // if close enough (tof dist_mm small enough), transition to grab state
    if (last_detection.ball_radius > 0 && last_detection.ball_radius <= BALL_STOP_TOF_MM) {
        ESP_LOGI(TAG, "Ball within reach (tof=%d mm)", last_detection.ball_radius);
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
        // compute base rotation from last known ball sideways offset
        // center_offset > 0 = right so base < 90, center_offset < 0 = left so base > 90
        float center_offset = (float)last_detection.ball_x - (CAM_FRAME_WIDTH / 2.0f);
        float angle_offset  = center_offset * (CAM_HFOV_DEG / (float)CAM_FRAME_WIDTH) * BASE_ANGLE_SCALE;
        float base_angle    = 90.0f - angle_offset;

        // clamp to min and max
        if (base_angle < BASE_ROTATION_MIN) base_angle = BASE_ROTATION_MIN;
        if (base_angle > BASE_ROTATION_MAX) base_angle = BASE_ROTATION_MAX;

        // send commands to arm to ensure servos are in ready grab pos
        arm_angles_t grab = arm_grab_ready;
        grab.base = base_angle;
        ESP_LOGI(TAG, "Moving arm to grab position (base=%.1f, px_offset=%.0f)",
                 base_angle, center_offset);
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
        for (int j = 2; j < 10; j++) {
            checksum ^= msg[j];
        }

        if (checksum != msg[10]) {
            ESP_LOGW(TAG, "UART checksum mismatch: got 0x%02X, expected 0x%02X",
                     msg[10], checksum);
            continue;
        }

        // parsing message into detection result struct
        result->detected    = (msg[3] != 0);
        result->ball_x      = (uint16_t)(msg[4] << 8) | msg[5];
        result->ball_y      = (uint16_t)(msg[6] << 8) | msg[7];
        result->ball_radius = (uint16_t)(msg[8] << 8) | msg[9];
        result->fresh       = true;

        return true;
    }

    return false;
}