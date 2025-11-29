package team3164.simulator.web;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import io.javalin.Javalin;
import io.javalin.websocket.WsContext;
import team3164.simulator.Constants;
import team3164.simulator.engine.InputState;
import team3164.simulator.engine.RobotState;
import team3164.simulator.engine.SimulationEngine;

import java.util.Map;
import java.util.concurrent.*;

/**
 * HTTP and WebSocket server for the simulator.
 *
 * Serves the web UI and handles real-time communication with the browser.
 */
public class SimulatorServer {

    private final SimulationEngine engine;
    private final Gson gson;
    private final int port;

    private Javalin app;
    private final Map<String, WsContext> clients = new ConcurrentHashMap<>();
    private ScheduledExecutorService broadcastExecutor;

    /**
     * Create a new simulator server.
     *
     * @param engine The simulation engine to use
     * @param port Port to listen on
     */
    public SimulatorServer(SimulationEngine engine, int port) {
        this.engine = engine;
        this.port = port;
        this.gson = new Gson();
    }

    /**
     * Start the server.
     */
    public void start() {
        app = Javalin.create(config -> {
            config.staticFiles.add(staticFiles -> {
                staticFiles.hostedPath = "/";
                staticFiles.directory = "/web";
                staticFiles.location = io.javalin.http.staticfiles.Location.CLASSPATH;
            });
            config.http.defaultContentType = "text/html";
        });

        // WebSocket endpoint for real-time communication
        app.ws("/ws", ws -> {
            ws.onConnect(ctx -> {
                String id = Integer.toHexString(ctx.hashCode());
                clients.put(id, ctx);
                System.out.println("Client connected: " + id + " (total: " + clients.size() + ")");

                // Send initial state
                sendState(ctx, engine.getState());
            });

            ws.onClose(ctx -> {
                String id = Integer.toHexString(ctx.hashCode());
                clients.remove(id);
                System.out.println("Client disconnected: " + id + " (total: " + clients.size() + ")");
            });

            ws.onMessage(ctx -> {
                try {
                    String message = ctx.message();
                    handleMessage(message);
                } catch (Exception e) {
                    System.err.println("Error handling message: " + e.getMessage());
                }
            });

            ws.onError(ctx -> {
                System.err.println("WebSocket error: " + ctx.error());
            });
        });

        // Health check endpoint
        app.get("/health", ctx -> {
            ctx.json(Map.of(
                "status", "ok",
                "running", engine.isRunning(),
                "clients", clients.size(),
                "ticks", engine.getTickCount()
            ));
        });

        // Start the server
        app.start(port);

        // Start broadcast loop
        startBroadcastLoop();

        System.out.println("Server started at http://localhost:" + port);
    }

    /**
     * Stop the server.
     */
    public void stop() {
        if (broadcastExecutor != null) {
            broadcastExecutor.shutdown();
        }

        if (app != null) {
            app.stop();
        }
    }

    /**
     * Start the periodic state broadcast loop.
     */
    private void startBroadcastLoop() {
        broadcastExecutor = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "StateBroadcast");
            t.setDaemon(true);
            return t;
        });

        long periodMs = 1000 / Constants.Simulation.BROADCAST_RATE;
        broadcastExecutor.scheduleAtFixedRate(() -> {
            try {
                broadcastState();
            } catch (Exception e) {
                System.err.println("Broadcast error: " + e.getMessage());
            }
        }, 0, periodMs, TimeUnit.MILLISECONDS);
    }

    /**
     * Broadcast current state to all connected clients.
     */
    private void broadcastState() {
        if (clients.isEmpty()) return;

        RobotState state = engine.getState();
        String json = buildStateJson(state);

        for (WsContext ctx : clients.values()) {
            try {
                ctx.send(json);
            } catch (Exception e) {
                // Client probably disconnected
            }
        }
    }

    /**
     * Send state to a single client.
     */
    private void sendState(WsContext ctx, RobotState state) {
        try {
            ctx.send(buildStateJson(state));
        } catch (Exception e) {
            System.err.println("Error sending state: " + e.getMessage());
        }
    }

    /**
     * Build JSON representation of robot state.
     */
    private String buildStateJson(RobotState state) {
        JsonObject json = new JsonObject();
        json.addProperty("type", "state");

        // Robot pose
        JsonObject robot = new JsonObject();
        robot.addProperty("x", round(state.x, 3));
        robot.addProperty("y", round(state.y, 3));
        robot.addProperty("heading", round(Math.toDegrees(state.heading), 1));
        robot.addProperty("vx", round(state.vx, 2));
        robot.addProperty("vy", round(state.vy, 2));
        robot.addProperty("omega", round(Math.toDegrees(state.omega), 1));
        json.add("robot", robot);

        // Swerve modules
        JsonObject swerve = new JsonObject();
        String[] moduleNames = {"fl", "fr", "rl", "rr"};
        for (int i = 0; i < 4; i++) {
            JsonObject module = new JsonObject();
            module.addProperty("angle", round(Math.toDegrees(state.moduleAngles[i]), 1));
            module.addProperty("speed", round(state.moduleSpeeds[i], 2));
            swerve.add(moduleNames[i], module);
        }
        json.add("swerve", swerve);

        // Elevator
        JsonObject elevator = new JsonObject();
        elevator.addProperty("height", round(state.getElevatorHeightInches(), 1));
        elevator.addProperty("goal", round(state.elevatorGoal * 39.37, 1));
        elevator.addProperty("velocity", round(state.elevatorVelocity * 39.37, 1));
        elevator.addProperty("atGoal", state.elevatorAtGoal);
        json.add("elevator", elevator);

        // Arm
        JsonObject arm = new JsonObject();
        arm.addProperty("angle", round(state.getArmAngleDegrees(), 1));
        arm.addProperty("goal", round(Math.toDegrees(state.armGoal), 1));
        arm.addProperty("atGoal", state.armAtGoal);
        json.add("arm", arm);

        // Claw
        JsonObject claw = new JsonObject();
        claw.addProperty("state", state.clawState.name());
        claw.addProperty("hasCoral", state.hasCoral);
        json.add("claw", claw);

        // Climber
        JsonObject climber = new JsonObject();
        climber.addProperty("position", round(state.climberPosition * 39.37, 1));
        json.add("climber", climber);

        // Control state
        JsonObject control = new JsonObject();
        control.addProperty("fieldRelative", state.fieldRelative);
        control.addProperty("slowMode", state.slowMode);
        control.addProperty("currentLevel", state.currentLevel);
        control.addProperty("command", state.currentCommand);
        json.add("control", control);

        // Game state
        JsonObject game = new JsonObject();
        game.addProperty("score", state.score);
        game.addProperty("time", round(state.matchTime, 1));
        game.addProperty("enabled", state.isEnabled);
        json.add("game", game);

        return json.toString();
    }

    /**
     * Handle incoming WebSocket message.
     */
    private void handleMessage(String message) {
        try {
            JsonObject json = gson.fromJson(message, JsonObject.class);
            String type = json.get("type").getAsString();

            if ("input".equals(type)) {
                InputState input = parseInput(json);
                engine.updateInput(input);
            } else if ("reset".equals(type)) {
                engine.getState().reset();
            } else if ("pickup".equals(type)) {
                // Debug: instantly pick up coral
                team3164.simulator.physics.ClawPhysics.pickupCoral(engine.getState());
            }
        } catch (Exception e) {
            System.err.println("Error parsing message: " + e.getMessage());
        }
    }

    /**
     * Parse input state from JSON.
     */
    private InputState parseInput(JsonObject json) {
        InputState input = new InputState();

        if (json.has("forward")) input.forward = json.get("forward").getAsDouble();
        if (json.has("strafe")) input.strafe = json.get("strafe").getAsDouble();
        if (json.has("turn")) input.turn = json.get("turn").getAsDouble();

        if (json.has("level0")) input.level0 = json.get("level0").getAsBoolean();
        if (json.has("level1")) input.level1 = json.get("level1").getAsBoolean();
        if (json.has("level2")) input.level2 = json.get("level2").getAsBoolean();
        if (json.has("level3")) input.level3 = json.get("level3").getAsBoolean();
        if (json.has("level4")) input.level4 = json.get("level4").getAsBoolean();

        if (json.has("intake")) input.intake = json.get("intake").getAsBoolean();
        if (json.has("outtake")) input.outtake = json.get("outtake").getAsBoolean();

        if (json.has("climberUp")) input.climberUp = json.get("climberUp").getAsBoolean();
        if (json.has("climberDown")) input.climberDown = json.get("climberDown").getAsBoolean();

        if (json.has("toggleSpeed")) input.toggleSpeed = json.get("toggleSpeed").getAsBoolean();
        if (json.has("toggleFieldRel")) input.toggleFieldRel = json.get("toggleFieldRel").getAsBoolean();
        if (json.has("resetGyro")) input.resetGyro = json.get("resetGyro").getAsBoolean();
        if (json.has("skiStop")) input.skiStop = json.get("skiStop").getAsBoolean();
        if (json.has("resetRobot")) input.resetRobot = json.get("resetRobot").getAsBoolean();

        return input;
    }

    /**
     * Round a double to specified decimal places.
     */
    private double round(double value, int places) {
        double scale = Math.pow(10, places);
        return Math.round(value * scale) / scale;
    }

    /**
     * Get the server port.
     */
    public int getPort() {
        return port;
    }
}
