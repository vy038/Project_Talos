/**
 * @file tutorial.js
 * @brief Interactive tutorial wizard for Lightsim
 */

function ensureExpanded(id) {
    const el = document.getElementById(id);
    if (el) el.classList.remove('collapsed');
    const title = document.querySelector(`.ctrl-title[data-target="${id}"]`);
    if(title) title.classList.remove('collapsed');
}

const tutorialSteps = [
    {
        title: "Welcome to Lightsim",
        text: "Welcome! This simulator lets you interact with my hexapod robot, Talos.<br><br>We're starting in a neutral state. When you launch normally, it auto-compiles the system, but we'll take it slow for now so we can look around.",
        target: null,
        softSpotlight: true,
        onEnter: () => {}
    },
    {
        title: "Viewports",
        text: "The Top Bar lets you manage your views. We have a 3D view, a 2D top-down view, an ESP-CAM view simulating the physical camera stream, and various hardware panels.",
        target: "#panel-toggles",
        softSpotlight: true,
        onEnter: () => {}
    },
    {
        title: "3D View Controls",
        text: "Let's go over how to move in the 3D space:<br>1. Click and drag to rotate around the robot.<br>2. Right click and drag to pan the camera.<br>3. Scroll up or down to zoom in and out.<br><br>Feel free to give it a try!",
        target: "#pane-view3d",
        softSpotlight: true,
        noScroll: true,
        onEnter: () => {}
    },
    {
        title: "Camera View",
        text: "This is the CAM view. It simulates what the physical ESP-CAM mounted on Talos sees, complete with computer vision filters mapping exactly to what the flight controller is analyzing.",
        target: "#pane-view-cam",
        softSpotlight: true,
        noScroll: true,
        onEnter: () => {},
        onExit: () => {}
    },
    {
        title: "Hardware Specifics",
        text: "This is the Debug Telemetry pane. It shows the 'brain' of the robot in real time. Here you'll see live sensor readouts, memory usage, RTOS task statuses, and raw terminal output directly from the flight controller.",
        target: "#pane-debug",
        softSpotlight: true,
        noScroll: true,
        onEnter: () => {}
    },
    {
        title: "Control Panel: System",
        text: "Here is the Control Panel. The SYSTEM section allows you to recompile and flash the firmware to the simulated robot, or hard test the backend by killing the simulator cleanly.",
        target: "#sec-system",
        softSpotlight: true,
        onEnter: () => { ensureExpanded('sec-system'); }
    },
    {
        title: "Control Panel: Physics Ball",
        text: "The PHYSICS BALL section lets you interact with the environment. You can toggle the ball and click the ground using the 'Place Ball' mode to drop it anywhere in the 3D space for the robot to detect.",
        target: "#sec-ball",
        softSpotlight: true,
        onEnter: () => { ensureExpanded('sec-ball'); }
    },
    {
        title: "Control Panel: Tests",
        text: "The TESTS section allows you to send isolated commands. Try running a gait test! Select 'All Gaits' from the dropdown and click 'Run Test'. Observe the robot's legs moving in the 3D view using the client-side gait generator.",
        target: "#sec-tests",
        softSpotlight: true,
        onEnter: () => { ensureExpanded('sec-tests'); }
    },
    {
        title: "Control Panel: Arm Control",
        text: "The ARM CONTROL section provides manual sliders to command the 3-DOF arm. You can manually adjust the joints, or enable IK Cursor Mode to drag a 3D target around using inverse kinematics!",
        target: "#sec-arm",
        softSpotlight: true,
        onEnter: () => { ensureExpanded('sec-arm'); }
    },
    {
        title: "Control Panel: Time Scale",
        text: "The TIME SCALE slider can speed up or slow down the physics and rendering engine! This is perfect for closely analyzing gait synchronization and kinematics in slow motion.",
        target: "#sec-timescale",
        softSpotlight: true,
        onEnter: () => { ensureExpanded('sec-timescale'); }
    },
    {
        title: "Control Panel: Movement",
        text: "The MOVEMENT section lets you manually drive the robot across the virtual floor, testing different gaits and walk/turn speeds.",
        target: "#sec-movement",
        softSpotlight: true,
        onEnter: () => { ensureExpanded('sec-movement'); }
    },
    {
        title: "Algorithm & State Machine",
        text: "Now for the fun part: running the full autonomous algorithm!<br><br>When you click Finish, I will compile the firmware automatically. Watch the System State indicator (top right) change from IDLE to SEARCH, APPROACH, and ALIGN as Talos tracks the object using the simulated camera.",
        target: "#state-badge",
        softSpotlight: true,
        noScroll: true,
        onEnter: () => {
            ensureExpanded('sec-system');
        }
    }
];

let currentTutorialStep = 0;
let tutorialActive = false;

const overlay = document.getElementById('tutorial-overlay');
const spotlight = document.getElementById('tutorial-spotlight');
const dialog = document.getElementById('tutorial-dialog');
const titleText = document.getElementById('tut-title-text');
const bodyText = document.getElementById('tut-body-text');
const progressText = document.getElementById('tut-progress');
const btnPrev = document.getElementById('tut-btn-prev');
const btnNext = document.getElementById('tut-btn-next');
const btnClose = document.getElementById('tut-btn-close');

function updateSpotlight(targetSelector) {
    if (!targetSelector) {
        spotlight.style.display = 'none';
        dialog.style.left = '50%';
        dialog.style.top = '50%';
        dialog.style.transform = 'translate(-50%, -50%)';
        return;
    }

    const el = document.querySelector(targetSelector);
    if (!el) {
        spotlight.style.display = 'none';
        return;
    }
    
    // Only scroll into view for ctrl sections (not top-level panes which don't need it and break layout)
    const step = tutorialSteps[currentTutorialStep];
    if (!step || !step.noScroll) {
        el.scrollIntoView({ block: 'nearest', inline: 'nearest' });
    }

    const rect = el.getBoundingClientRect();
    const pad = 8;
    spotlight.style.display = 'block';
    spotlight.style.left = (rect.left - pad) + 'px';
    spotlight.style.top = (rect.top - pad) + 'px';
    spotlight.style.width = (rect.width + pad * 2) + 'px';
    spotlight.style.height = (rect.height + pad * 2) + 'px';

    // Position dialog relative to spotlight
    const dialogRect = dialog.getBoundingClientRect();
    
    // Default to right of the element
    let dLeft = rect.right + pad + 20;
    let dTop = rect.top;
    
    // If it falls off screen to the right, put it on the left
    if (dLeft + dialogRect.width > window.innerWidth) {
        dLeft = rect.left - pad - 20 - dialogRect.width;
    }
    
    // If it's too high up, clamp it
    if (dTop < 20) dTop = 20;
    
    // If it falls off screen to bottom, clamp it
    if (dTop + dialogRect.height > window.innerHeight) {
        dTop = window.innerHeight - dialogRect.height - 20;
    }

    // Apply soft layout if requested
    if (tutorialSteps[currentTutorialStep] && tutorialSteps[currentTutorialStep].softSpotlight) {
        spotlight.classList.add('soft');
    } else {
        spotlight.classList.remove('soft');
    }

    dialog.style.left = dLeft + 'px';
    dialog.style.top = dTop + 'px';
    dialog.style.transform = 'none';
}

function showTutorialStep(idx) {
    if (idx < 0 || idx >= tutorialSteps.length) return;
    
    if (tutorialSteps[currentTutorialStep] && tutorialSteps[currentTutorialStep].onExit) {
        tutorialSteps[currentTutorialStep].onExit();
    }
    
    currentTutorialStep = idx;
    
    const step = tutorialSteps[idx];
    titleText.textContent = step.title;
    bodyText.innerHTML = step.text;
    progressText.textContent = `${idx + 1} / ${tutorialSteps.length}`;
    
    btnPrev.disabled = (idx === 0);
    btnNext.textContent = (idx === tutorialSteps.length - 1) ? 'Finish' : 'Next';
    
    if (step.onEnter) step.onEnter();
    
    // Slight delay for UI recalculations before setting spotlight
    setTimeout(() => {
        updateSpotlight(step.target);
    }, 50);
}

function startTutorial() {
    document.getElementById('splash-screen').classList.add('hidden');
    document.getElementById('app').classList.remove('hidden');
    
    // Trigger a resize event to ensure canvases render correctly after being hidden
    window.dispatchEvent(new Event('resize'));

    // Reset robot joints to neutral standing pose and clear any old sim state
    if (typeof clearAll === 'function') clearAll();
    if (window.resetRobotPosition) window.resetRobotPosition();
    
    tutorialActive = true;
    overlay.classList.add('active');
    dialog.classList.add('active');
    
    showTutorialStep(0);
}

function endTutorial() {
    tutorialActive = false;
    overlay.classList.remove('active');
    dialog.classList.remove('active');
    spotlight.style.display = 'none';

    // If we just finished the final step, auto-compile and drop ball
    if (currentTutorialStep === tutorialSteps.length - 1) {
        document.getElementById('btn-recompile').click(); 
        
        setTimeout(() => {
            if (window.PhysicsBall && window.PhysicsBall.drop) {
                window.PhysicsBall.drop(240, 120);
            }
        }, 3000);
    }
}

function nextStep() {
    if (currentTutorialStep < tutorialSteps.length - 1) {
        showTutorialStep(currentTutorialStep + 1);
    } else {
        endTutorial();
    }
}

function prevStep() {
    if (currentTutorialStep > 0) {
        showTutorialStep(currentTutorialStep - 1);
    }
}

// Event Listeners
btnNext.addEventListener('click', nextStep);
btnPrev.addEventListener('click', prevStep);
btnClose.addEventListener('click', endTutorial);

document.getElementById('btn-tutorial').addEventListener('click', startTutorial);
document.getElementById('btn-restart-tutorial').addEventListener('click', startTutorial);

// Handle window resize dynamically to keep spotlight in place
window.addEventListener('resize', () => {
    if (tutorialActive) {
        updateSpotlight(tutorialSteps[currentTutorialStep].target);
    }
});
