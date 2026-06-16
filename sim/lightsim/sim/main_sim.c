/**
 * @file main_sim.c
 * @brief Lightsim entry point - wraps firmware's app_main()
 *
 * Initializes the simulation state system, then calls the real firmware
 * entry point. The firmware runs against HAL stubs that capture state
 * and emit JSON for the visualization frontend.
 *
 * stdout = JSON state lines (for bridge server)
 * stderr = firmware logs and debug output
 *
 * Injection server: TCP port 9998 (loopback only)
 *   Accepts one client at a time. Protocol is identical to the old stdin:
 *     - 0xAA ... (13 bytes): binary UART detection packet → sim_uart_inject
 *     - { ... }\n          : JSON line, fields are checked independently:
 *         "mm"        → sim_set_tof_distance_mm (ToF override)
 *         "command"   → D-pad/keyboard move command: kills the autonomous
 *                        state-machine task and drives the gait generator directly
 *         "timescale" → sim_set_time_scale (backend FreeRTOS task speed)
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include "sim_state.h"
#include "gait_generator.h"

#define INJECTION_PORT_DEFAULT 9998

/* Forward declarations of firmware entry point and state getter */
extern void app_main(void);
extern int  xStateMachineGetState(void);

/* sim_uart_inject is defined in uart_stub.c */
extern void sim_uart_inject(const uint8_t *data, size_t len);

/* sim_set_tof_distance_mm is defined in sim_state.c */
extern void sim_set_tof_distance_mm(uint16_t mm);

/* sim_suspend_task_by_name is defined in freertos_sim.c */
extern void sim_suspend_task_by_name(const char *name);

/* Map frontend "command" strings (sendRobotCommand's data-cmd) to gait commands */
static bool move_command_from_str(const char *s, move_command_t *out) {
    if (!strncmp(s, "forward", 7))    { *out = MOVE_FORWARD;   return true; }
    if (!strncmp(s, "backward", 8))   { *out = MOVE_BACKWARD;  return true; }
    if (!strncmp(s, "turn_left", 9))  { *out = MOVE_TURN_LEFT; return true; }
    if (!strncmp(s, "turn_right", 10)){ *out = MOVE_TURN_RIGHT; return true; }
    if (!strncmp(s, "stop", 4))       { *out = MOVE_STOP;      return true; }
    return false;
}

/* Parse and dispatch a buffer of bytes received from the injection client */
static void process_injection_buf(uint8_t *buf, size_t *buf_len) {
    size_t i = 0;
    while (i < *buf_len) {
        if (buf[i] == 0xAA) {
            /* Binary UART packet — needs 13 bytes */
            if (*buf_len - i < 13) break;
            sim_uart_inject(buf + i, 13);
            i += 13;
        } else if (buf[i] == '{') {
            /* JSON line — find newline */
            size_t j = i;
            while (j < *buf_len && buf[j] != '\n') j++;
            if (j >= *buf_len) break; /* incomplete, wait */
            buf[j] = '\0';
            char *line = (char *)(buf + i);

            char *mm_ptr = strstr(line, "\"mm\":");
            if (mm_ptr) {
                int mm_val = atoi(mm_ptr + 5);
                if (mm_val > 0 && mm_val <= 8190)
                    sim_set_tof_distance_mm((uint16_t)mm_val);
            }

            /* {"type":"move","command":"forward","speed":0.5,"gait":0} — D-pad/keyboard
             * commands. Any movement command hands gait control to teleop and stops
             * the autonomous program so it doesn't fight with manual driving. */
            char *cmd_ptr = strstr(line, "\"command\":\"");
            if (cmd_ptr) {
                cmd_ptr += strlen("\"command\":\"");
                move_command_t move;
                if (move_command_from_str(cmd_ptr, &move)) {
                    /* kill the autonomous program so it stops issuing its own
                     * gait commands and fighting with teleop */
                    sim_suspend_task_by_name("state_mach");

                    char *gait_ptr = strstr(line, "\"gait\":");
                    if (gait_ptr) vGaitSetType((gait_type_t)atoi(gait_ptr + 7));

                    float speed = 0.5f;
                    char *speed_ptr = strstr(line, "\"speed\":");
                    if (speed_ptr) speed = (float)atof(speed_ptr + 8);

                    vGaitSetCommand(move, speed);
                }
            }

            /* {"timescale":1.5} — scales backend FreeRTOS task timing */
            char *ts_ptr = strstr(line, "\"timescale\":");
            if (ts_ptr) {
                sim_set_time_scale((float)atof(ts_ptr + strlen("\"timescale\":")));
            }

            i = j + 1;
        } else {
            i++;
        }
    }
    if (i > 0 && i < *buf_len) {
        memmove(buf, buf + i, *buf_len - i);
        *buf_len -= i;
    } else if (i >= *buf_len) {
        *buf_len = 0;
    }
}

/*
 * TCP injection server thread.
 * Listens on 127.0.0.1:<port>, accepts one client at a time, reads
 * injection bytes and dispatches them. Port passed via arg.
 */
static void *injection_server_thread(void *arg) {
    int port = (int)(uintptr_t)arg;

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) {
        fprintf(stderr, "[INJ] socket() failed: %s\n", strerror(errno));
        return NULL;
    }

    int opt = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_LOOPBACK),
        .sin_port        = htons((uint16_t)port),
    };
    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        fprintf(stderr, "[INJ] bind() failed: %s\n", strerror(errno));
        close(srv);
        return NULL;
    }
    listen(srv, 1);
    fprintf(stderr, "[INJ] Injection server ready on 127.0.0.1:%d\n", port);
    fflush(stderr);

    while (1) {
        int cli = accept(srv, NULL, NULL);
        if (cli < 0) continue;
        fprintf(stderr, "[INJ] Client connected\n");
        fflush(stderr);

        uint8_t buf[512];
        size_t  buf_len = 0;

        while (1) {
            ssize_t n = read(cli, buf + buf_len, sizeof(buf) - buf_len - 1);
            if (n <= 0) break;
            buf_len += (size_t)n;
            process_injection_buf(buf, &buf_len);
        }

        fprintf(stderr, "[INJ] Client disconnected\n");
        fflush(stderr);
        close(cli);
    }

    close(srv);
    return NULL;
}

static volatile int running = 1;

static void sigint_handler(int sig) {
    (void)sig;
    running = 0;
    sim_request_exit();
}

int main(int argc, char *argv[]) {
    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    /* Disable output buffering so JSON lines appear immediately */
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    /* Parse flags */
    bool json_mode = false;
    int  injection_port = INJECTION_PORT_DEFAULT;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--json") == 0) {
            json_mode = true;
        } else if (strcmp(argv[i], "--inj-port") == 0 && i + 1 < argc) {
            injection_port = atoi(argv[++i]);
        }
    }

    if (json_mode) {
        sim_enable_json_output(true);
    }

    fprintf(stderr, "=== Lightsim - Project Talos Simulator ===\n");
    fprintf(stderr, "JSON output: %s\n", json_mode ? "ON (stdout)" : "OFF");
    fprintf(stderr, "Starting firmware simulation...\n\n");

    /* Initialize simulation state */
    sim_state_init();

    /* Start TCP injection server — replaces stdin pipe (which had buffering issues) */
    pthread_t inj_thread;
    pthread_create(&inj_thread, NULL, injection_server_thread, (void *)(uintptr_t)injection_port);
    pthread_detach(inj_thread);

    /* Bridge: let sim_state poll robot state without including firmware headers */
    sim_register_state_getter(xStateMachineGetState);

    /* Run the firmware entry point */
    app_main();

    /* app_main returns after spawning tasks, keep the process alive so the
     * pthread-backed FreeRTOS tasks continue running, matching real RTOS behaviour
     * where the scheduler keeps running after app_main's task exits. */
    while (running) {
        usleep(100000);
    }

    fprintf(stderr, "\nSimulation ended.\n");
    return 0;
}
