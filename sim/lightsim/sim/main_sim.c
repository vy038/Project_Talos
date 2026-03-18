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
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include "sim_state.h"

/* Forward declaration of the firmware entry point */
extern void app_main(void);

static volatile int running = 1;

static void sigint_handler(int sig) {
    (void)sig;
    running = 0;
    sim_request_exit();
}

int main(int argc, char *argv[]) {
    (void)argc; (void)argv;

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    /* Disable output buffering so JSON lines appear immediately */
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);

    /* Check for --json flag to enable JSON state emission on stdout */
    bool json_mode = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--json") == 0) {
            json_mode = true;
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

    /* Run the firmware entry point */
    app_main();

    fprintf(stderr, "\nSimulation ended.\n");
    return 0;
}
