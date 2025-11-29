/**
 * Reefscape 2025 Simulator - Web Frontend
 * Team 3164 Stealth Tigers
 */

// ============================================================================
// CONSTANTS
// ============================================================================

const FIELD = {
    LENGTH: 16.4592,  // meters (54 feet)
    WIDTH: 8.2296,    // meters (27 feet)
    REEF_CENTER_X: 16.4592 / 2,
    REEF_CENTER_Y: 8.2296 / 2,
    REEF_RADIUS: 0.9144  // 36 inches
};

const ROBOT = {
    LENGTH: 0.9334,  // with bumpers
    WIDTH: 0.9334
};

// ============================================================================
// STATE
// ============================================================================

let ws = null;
let connected = false;
let state = null;
let canvas, ctx;

// Input state
const keys = {};
const input = {
    forward: 0,
    strafe: 0,
    turn: 0,
    level0: false,
    level1: false,
    level2: false,
    level3: false,
    level4: false,
    intake: false,
    outtake: false,
    climberUp: false,
    climberDown: false,
    toggleSpeed: false,
    toggleFieldRel: false,
    resetGyro: false,
    skiStop: false,
    resetRobot: false
};

// ============================================================================
// WEBSOCKET
// ============================================================================

function connect() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl = `${protocol}//${window.location.host}/ws`;

    ws = new WebSocket(wsUrl);

    ws.onopen = () => {
        connected = true;
        updateConnectionStatus(true);
        console.log('Connected to simulator');
    };

    ws.onclose = () => {
        connected = false;
        updateConnectionStatus(false);
        console.log('Disconnected from simulator');
        // Reconnect after delay
        setTimeout(connect, 2000);
    };

    ws.onerror = (error) => {
        console.error('WebSocket error:', error);
    };

    ws.onmessage = (event) => {
        try {
            const data = JSON.parse(event.data);
            if (data.type === 'state') {
                state = data;
                updateUI();
            }
        } catch (e) {
            console.error('Error parsing message:', e);
        }
    };
}

function sendInput() {
    if (!connected || !ws) return;

    // Update continuous inputs from keys
    input.forward = (keys['KeyW'] ? 1 : 0) - (keys['KeyS'] ? 1 : 0);
    input.strafe = (keys['KeyA'] ? 1 : 0) - (keys['KeyD'] ? 1 : 0);
    input.turn = (keys['KeyQ'] ? 1 : 0) - (keys['KeyE'] ? 1 : 0);

    input.level0 = keys['KeyR'] || false;
    input.level1 = keys['Digit1'] || false;
    input.level2 = keys['Digit2'] || false;
    input.level3 = keys['Digit3'] || false;
    input.level4 = keys['Digit4'] || false;

    input.intake = keys['Space'] || false;
    input.outtake = keys['ShiftLeft'] || keys['ShiftRight'] || false;

    input.climberUp = keys['ArrowUp'] || false;
    input.climberDown = keys['ArrowDown'] || false;

    input.skiStop = keys['KeyV'] || false;

    try {
        ws.send(JSON.stringify({
            type: 'input',
            ...input
        }));
    } catch (e) {
        console.error('Error sending input:', e);
    }

    // Reset one-shot inputs
    input.toggleSpeed = false;
    input.toggleFieldRel = false;
    input.resetGyro = false;
    input.resetRobot = false;
}

// ============================================================================
// INPUT HANDLING
// ============================================================================

function setupInputHandlers() {
    document.addEventListener('keydown', (e) => {
        if (e.repeat) return;

        keys[e.code] = true;

        // One-shot actions
        switch (e.code) {
            case 'KeyX':
                input.toggleSpeed = true;
                break;
            case 'KeyC':
                input.toggleFieldRel = true;
                break;
            case 'KeyG':
                input.resetGyro = true;
                break;
            case 'Escape':
                input.resetRobot = true;
                break;
            case 'KeyP':
                // Debug: pickup coral
                if (ws && connected) {
                    ws.send(JSON.stringify({ type: 'pickup' }));
                }
                break;
        }

        // Prevent default for game keys
        if (['Space', 'ArrowUp', 'ArrowDown'].includes(e.code)) {
            e.preventDefault();
        }
    });

    document.addEventListener('keyup', (e) => {
        keys[e.code] = false;
    });

    // Lose focus handler
    window.addEventListener('blur', () => {
        Object.keys(keys).forEach(k => keys[k] = false);
    });
}

// ============================================================================
// UI UPDATES
// ============================================================================

function updateConnectionStatus(isConnected) {
    const el = document.getElementById('connection-status');
    if (isConnected) {
        el.textContent = 'Connected';
        el.className = 'status connected';
    } else {
        el.textContent = 'Disconnected';
        el.className = 'status disconnected';
    }
}

function updateUI() {
    if (!state) return;

    // Match time
    const time = state.game.time || 0;
    const mins = Math.floor(time / 60);
    const secs = Math.floor(time % 60);
    document.getElementById('match-time').textContent =
        `${mins}:${secs.toString().padStart(2, '0')}`;

    // Score
    document.getElementById('score').textContent = `Score: ${state.game.score}`;

    // Elevator
    const elevHeight = state.elevator.height;
    const elevGoal = state.elevator.goal;
    const elevMin = 30.5;
    const elevMax = 78.5;
    const elevPercent = ((elevHeight - elevMin) / (elevMax - elevMin)) * 100;
    const elevGoalPercent = ((elevGoal - elevMin) / (elevMax - elevMin)) * 100;

    document.getElementById('elevator-value').textContent = `${elevHeight.toFixed(1)}"`;
    document.getElementById('elevator-bar').style.width = `${elevPercent}%`;
    document.getElementById('elevator-goal').style.left = `${elevGoalPercent}%`;

    // Arm
    const armAngle = state.arm.angle;
    document.getElementById('arm-value').textContent = `${armAngle.toFixed(0)}°`;
    document.getElementById('arm-indicator').style.transform =
        `translateY(-50%) rotate(${-armAngle}deg)`;

    // Claw
    const clawState = state.claw.state.toLowerCase();
    const clawEl = document.getElementById('claw-indicator');
    clawEl.className = `claw-indicator ${clawState}`;
    document.getElementById('claw-value').textContent = state.claw.state;

    // Climber
    const climberPos = state.climber.position;
    const climberPercent = (climberPos / 24) * 100;
    document.getElementById('climber-value').textContent = `${climberPos.toFixed(1)}"`;
    document.getElementById('climber-bar').style.width = `${climberPercent}%`;

    // Robot info
    document.getElementById('robot-position').textContent =
        `(${state.robot.x.toFixed(1)}, ${state.robot.y.toFixed(1)})`;
    document.getElementById('robot-heading').textContent = `${state.robot.heading.toFixed(0)}°`;

    const speed = Math.hypot(state.robot.vx, state.robot.vy);
    document.getElementById('robot-speed').textContent = `${speed.toFixed(1)} m/s`;

    document.getElementById('current-level').textContent = `L${state.control.currentLevel}`;
    document.getElementById('current-command').textContent = state.control.command;

    // Mode indicators
    document.getElementById('mode-field-rel').className =
        `mode-indicator ${state.control.fieldRelative ? 'active' : ''}`;
    document.getElementById('mode-slow').className =
        `mode-indicator ${state.control.slowMode ? 'active' : ''}`;

    // Render field
    renderField();
}

// ============================================================================
// FIELD RENDERING
// ============================================================================

function initCanvas() {
    canvas = document.getElementById('field-canvas');
    ctx = canvas.getContext('2d');

    // Handle resize
    function resize() {
        const container = canvas.parentElement;
        const width = container.clientWidth - 20;
        const height = width * (FIELD.WIDTH / FIELD.LENGTH);

        canvas.width = width;
        canvas.height = height;

        if (state) renderField();
    }

    window.addEventListener('resize', resize);
    resize();
}

function renderField() {
    if (!ctx || !state) return;

    const w = canvas.width;
    const h = canvas.height;

    // Scale factor
    const scale = w / FIELD.LENGTH;

    // Clear
    ctx.fillStyle = '#1e3a5f';
    ctx.fillRect(0, 0, w, h);

    // Field carpet texture
    ctx.fillStyle = '#2d4a6f';
    for (let i = 0; i < 20; i++) {
        ctx.fillRect(i * (w / 20), 0, 1, h);
    }

    // Field border
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.strokeRect(2, 2, w - 4, h - 4);

    // Center line
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)';
    ctx.setLineDash([10, 5]);
    ctx.beginPath();
    ctx.moveTo(w / 2, 0);
    ctx.lineTo(w / 2, h);
    ctx.stroke();
    ctx.setLineDash([]);

    // Draw reef (hexagon)
    const reefX = FIELD.REEF_CENTER_X * scale;
    const reefY = h - (FIELD.REEF_CENTER_Y * scale);  // Flip Y
    const reefR = FIELD.REEF_RADIUS * scale;

    ctx.fillStyle = 'rgba(0, 212, 255, 0.2)';
    ctx.strokeStyle = '#00d4ff';
    ctx.lineWidth = 3;
    ctx.beginPath();
    for (let i = 0; i < 6; i++) {
        const angle = (i * 60 - 30) * Math.PI / 180;
        const x = reefX + reefR * Math.cos(angle);
        const y = reefY + reefR * Math.sin(angle);
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
    }
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // Reef center
    ctx.fillStyle = '#00d4ff';
    ctx.beginPath();
    ctx.arc(reefX, reefY, 5, 0, Math.PI * 2);
    ctx.fill();

    // Draw scoring positions (A-L)
    const positions = 'ABCDEFGHIJKL'.split('');
    for (let i = 0; i < 12; i++) {
        const pipeOffset = (i % 2 === 0) ? 1 : -1;
        const faceAngle = Math.floor(i / 2) * 60;

        const baseAngle = (faceAngle - 90) * Math.PI / 180;
        const perpAngle = baseAngle + Math.PI / 2;

        const dist = reefR * 1.3;
        const lateralOffset = 0.15 * scale * pipeOffset;

        const px = reefX + dist * Math.cos(baseAngle) + lateralOffset * Math.cos(perpAngle);
        const py = reefY + dist * Math.sin(baseAngle) + lateralOffset * Math.sin(perpAngle);

        ctx.fillStyle = 'rgba(255, 215, 0, 0.5)';
        ctx.beginPath();
        ctx.arc(px, py, 8, 0, Math.PI * 2);
        ctx.fill();

        ctx.fillStyle = '#fff';
        ctx.font = '10px sans-serif';
        ctx.textAlign = 'center';
        ctx.textBaseline = 'middle';
        ctx.fillText(positions[i], px, py);
    }

    // Draw coral stations
    ctx.fillStyle = 'rgba(0, 168, 120, 0.3)';
    ctx.fillRect(0, h - 2 * scale, 1 * scale, 2 * scale);
    ctx.fillRect(0, 0, 1 * scale, 2 * scale);

    // Draw robot
    drawRobot(state.robot.x * scale, h - (state.robot.y * scale),
              -state.robot.heading * Math.PI / 180, scale);

    // Draw swerve modules
    drawSwerveModules(state.robot.x * scale, h - (state.robot.y * scale),
                      -state.robot.heading * Math.PI / 180, scale);
}

function drawRobot(x, y, heading, scale) {
    const robotW = ROBOT.WIDTH * scale;
    const robotH = ROBOT.LENGTH * scale;

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(heading);

    // Robot body
    ctx.fillStyle = state.claw.hasCoral ? '#00a878' : '#e94560';
    ctx.fillRect(-robotW / 2, -robotH / 2, robotW, robotH);

    // Robot outline
    ctx.strokeStyle = '#fff';
    ctx.lineWidth = 2;
    ctx.strokeRect(-robotW / 2, -robotH / 2, robotW, robotH);

    // Direction arrow
    ctx.fillStyle = '#fff';
    ctx.beginPath();
    ctx.moveTo(0, -robotH / 2 - 5);
    ctx.lineTo(-8, -robotH / 2 + 10);
    ctx.lineTo(8, -robotH / 2 + 10);
    ctx.closePath();
    ctx.fill();

    // Team number
    ctx.fillStyle = '#fff';
    ctx.font = 'bold 12px sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillText('3164', 0, 0);

    ctx.restore();
}

function drawSwerveModules(x, y, heading, scale) {
    if (!state.swerve) return;

    const halfW = ROBOT.WIDTH * scale / 2 - 5;
    const halfH = ROBOT.LENGTH * scale / 2 - 5;

    const modulePositions = [
        { x: halfH, y: halfW },   // FL
        { x: halfH, y: -halfW },  // FR
        { x: -halfH, y: halfW },  // RL
        { x: -halfH, y: -halfW }  // RR
    ];

    const modules = ['fl', 'fr', 'rl', 'rr'];

    ctx.save();
    ctx.translate(x, y);
    ctx.rotate(heading);

    modules.forEach((name, i) => {
        const mod = state.swerve[name];
        const pos = modulePositions[i];

        ctx.save();
        ctx.translate(pos.x, pos.y);
        ctx.rotate(-mod.angle * Math.PI / 180);

        // Wheel
        const wheelLength = 15;
        const wheelWidth = 6;

        ctx.fillStyle = '#333';
        ctx.fillRect(-wheelLength / 2, -wheelWidth / 2, wheelLength, wheelWidth);

        // Speed indicator
        const speedScale = Math.min(Math.abs(mod.speed) / 4, 1);
        ctx.strokeStyle = mod.speed >= 0 ? '#0f0' : '#f00';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(speedScale * 15 * Math.sign(mod.speed || 1), 0);
        ctx.stroke();

        ctx.restore();
    });

    ctx.restore();
}

// ============================================================================
// INITIALIZATION
// ============================================================================

function init() {
    initCanvas();
    setupInputHandlers();
    connect();

    // Send input at 50Hz
    setInterval(sendInput, 20);

    console.log('Reefscape 2025 Simulator initialized');
}

// Start when DOM is ready
if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
} else {
    init();
}
