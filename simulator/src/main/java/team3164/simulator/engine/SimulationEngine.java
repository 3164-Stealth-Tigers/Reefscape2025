package team3164.simulator.engine;

import team3164.simulator.Constants;
import team3164.simulator.physics.*;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;

/**
 * Core simulation engine that runs the physics loop.
 *
 * Runs at 50Hz (20ms per tick) to match WPILib robot code timing.
 * Updates all subsystem physics and broadcasts state changes.
 */
public class SimulationEngine {

    private final RobotState state;
    private final InputState input;

    private ScheduledExecutorService executor;
    private Consumer<RobotState> stateListener;
    private Consumer<String> logListener;

    private boolean running = false;
    private long tickCount = 0;
    private long startTimeMs;

    /**
     * Create a new simulation engine.
     */
    public SimulationEngine() {
        this.state = new RobotState();
        this.input = new InputState();
    }

    /**
     * Start the simulation loop.
     */
    public void start() {
        if (running) return;

        running = true;
        startTimeMs = System.currentTimeMillis();
        tickCount = 0;

        executor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "SimulationEngine");
            t.setDaemon(true);
            return t;
        });

        long periodMs = (long) (1000.0 / Constants.Simulation.TICK_RATE);
        executor.scheduleAtFixedRate(this::tick, 0, periodMs, TimeUnit.MILLISECONDS);

        log("Simulation started");
    }

    /**
     * Stop the simulation loop.
     */
    public void stop() {
        if (!running) return;

        running = false;
        if (executor != null) {
            executor.shutdown();
            try {
                executor.awaitTermination(1, TimeUnit.SECONDS);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        log("Simulation stopped");
    }

    /**
     * Single simulation tick - called at 50Hz.
     */
    private void tick() {
        try {
            double dt = Constants.Simulation.DT;
            tickCount++;

            // Update match time
            state.matchTime = (System.currentTimeMillis() - startTimeMs) / 1000.0;

            // Process mode toggles (edge detection would be better, but simplified here)
            processToggles();

            // Process level requests
            processLevelRequests();

            // Update all physics
            SwervePhysics.update(state, input, dt);
            ElevatorPhysics.update(state, dt);
            ArmPhysics.update(state, dt);
            ClimberPhysics.update(state, input, dt);

            boolean scored = ClawPhysics.update(state, input, dt);
            if (scored) {
                handleScore();
            }

            // Update command name based on current actions
            updateCommandName();

            // Notify listeners
            if (stateListener != null) {
                stateListener.accept(state);
            }

        } catch (Exception e) {
            log("Error in simulation tick: " + e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * Process toggle button inputs.
     */
    private void processToggles() {
        if (input.resetGyro) {
            SwervePhysics.resetGyro(state, 180);
            input.resetGyro = false;  // Consume the input
            log("Gyro reset to 180°");
        }

        if (input.toggleFieldRel) {
            state.fieldRelative = !state.fieldRelative;
            input.toggleFieldRel = false;
            log("Field-relative: " + (state.fieldRelative ? "ON" : "OFF"));
        }

        if (input.toggleSpeed) {
            state.slowMode = !state.slowMode;
            input.toggleSpeed = false;
            log("Slow mode: " + (state.slowMode ? "ON" : "OFF"));
        }

        if (input.resetRobot) {
            state.reset();
            input.resetRobot = false;
            log("Robot state reset");
        }
    }

    /**
     * Process level selection requests.
     */
    private void processLevelRequests() {
        int level = input.getRequestedLevel();
        if (level >= 0 && level != state.currentLevel) {
            ElevatorPhysics.setLevel(state, level);
            ArmPhysics.setLevel(state, level);
            log("Set level " + level + " (height=" +
                String.format("%.1f", state.elevatorGoal * 39.37) + "\", angle=" +
                String.format("%.0f", Math.toDegrees(state.armGoal)) + "°)");
        }
    }

    /**
     * Update the current command name for display.
     */
    private void updateCommandName() {
        if (!state.elevatorAtGoal) {
            state.currentCommand = "Moving to L" + state.currentLevel;
        } else if (!state.armAtGoal) {
            state.currentCommand = "Arm moving";
        } else if (state.clawState == RobotState.ClawState.INTAKING) {
            state.currentCommand = "Intaking";
        } else if (state.clawState == RobotState.ClawState.OUTTAKING) {
            state.currentCommand = "Scoring";
        } else if (state.hasCoral) {
            state.currentCommand = "Holding coral";
        } else if (input.hasDriveInput()) {
            state.currentCommand = "Driving";
        } else {
            state.currentCommand = "Idle";
        }
    }

    /**
     * Handle scoring a coral piece.
     */
    private void handleScore() {
        int points;
        switch (state.currentLevel) {
            case 1: points = 2; break;
            case 2: points = 3; break;
            case 3: points = 4; break;
            case 4: points = 6; break;
            default: points = 0;
        }

        state.score += points;
        log("SCORED! Level " + state.currentLevel + " (+" + points + " pts, total: " + state.score + ")");
    }

    /**
     * Update input state from external source (WebSocket).
     */
    public void updateInput(InputState newInput) {
        synchronized (input) {
            input.forward = newInput.forward;
            input.strafe = newInput.strafe;
            input.turn = newInput.turn;

            input.level0 = newInput.level0;
            input.level1 = newInput.level1;
            input.level2 = newInput.level2;
            input.level3 = newInput.level3;
            input.level4 = newInput.level4;

            input.intake = newInput.intake;
            input.outtake = newInput.outtake;

            input.climberUp = newInput.climberUp;
            input.climberDown = newInput.climberDown;

            // Edge-triggered inputs
            if (newInput.toggleSpeed) input.toggleSpeed = true;
            if (newInput.toggleFieldRel) input.toggleFieldRel = true;
            if (newInput.resetGyro) input.resetGyro = true;
            input.skiStop = newInput.skiStop;
            if (newInput.resetRobot) input.resetRobot = true;
        }
    }

    /**
     * Get current robot state (for broadcasting).
     */
    public RobotState getState() {
        return state;
    }

    /**
     * Set listener for state updates.
     */
    public void setStateListener(Consumer<RobotState> listener) {
        this.stateListener = listener;
    }

    /**
     * Set listener for log messages.
     */
    public void setLogListener(Consumer<String> listener) {
        this.logListener = listener;
    }

    /**
     * Log a message to console and listeners.
     */
    private void log(String message) {
        String timestamp = String.format("[%02d:%02d:%02d]",
            (int)(state.matchTime / 3600) % 24,
            (int)(state.matchTime / 60) % 60,
            (int)state.matchTime % 60);

        String fullMessage = timestamp + " " + message;
        System.out.println(fullMessage);

        if (logListener != null) {
            logListener.accept(fullMessage);
        }
    }

    /**
     * Check if simulation is running.
     */
    public boolean isRunning() {
        return running;
    }

    /**
     * Get tick count for diagnostics.
     */
    public long getTickCount() {
        return tickCount;
    }
}
