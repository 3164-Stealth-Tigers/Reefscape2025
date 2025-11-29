package team3164.simulator;

import team3164.simulator.engine.SimulationEngine;
import team3164.simulator.web.SimulatorServer;

import java.awt.Desktop;
import java.net.URI;

/**
 * Main entry point for the Reefscape 2025 Simulator.
 *
 * Usage:
 *   java -jar reefscape-sim.jar [options]
 *
 * Options:
 *   --port <port>    Server port (default: 8080)
 *   --no-browser     Don't auto-open browser
 *   --help           Show this help
 */
public class Main {

    private static final String BANNER = """

    ╔═══════════════════════════════════════════════════════════════════╗
    ║                                                                   ║
    ║   ██████╗ ███████╗███████╗███████╗███████╗ ██████╗ █████╗ ██████╗ ███████╗  ║
    ║   ██╔══██╗██╔════╝██╔════╝██╔════╝██╔════╝██╔════╝██╔══██╗██╔══██╗██╔════╝  ║
    ║   ██████╔╝█████╗  █████╗  █████╗  ███████╗██║     ███████║██████╔╝█████╗    ║
    ║   ██╔══██╗██╔══╝  ██╔══╝  ██╔══╝  ╚════██║██║     ██╔══██║██╔═══╝ ██╔══╝    ║
    ║   ██║  ██║███████╗███████╗██║     ███████║╚██████╗██║  ██║██║     ███████╗  ║
    ║   ╚═╝  ╚═╝╚══════╝╚══════╝╚═╝     ╚══════╝ ╚═════╝╚═╝  ╚═╝╚═╝     ╚══════╝  ║
    ║                                                                   ║
    ║                    2025 SIMULATOR - TEAM 3164                     ║
    ║                       STEALTH TIGERS                              ║
    ║                                                                   ║
    ╚═══════════════════════════════════════════════════════════════════╝
    """;

    private static final int DEFAULT_PORT = 8080;

    public static void main(String[] args) {
        // Parse arguments
        int port = DEFAULT_PORT;
        boolean openBrowser = true;

        for (int i = 0; i < args.length; i++) {
            switch (args[i]) {
                case "--port":
                case "-p":
                    if (i + 1 < args.length) {
                        try {
                            port = Integer.parseInt(args[++i]);
                        } catch (NumberFormatException e) {
                            System.err.println("Invalid port number: " + args[i]);
                            System.exit(1);
                        }
                    }
                    break;

                case "--no-browser":
                    openBrowser = false;
                    break;

                case "--help":
                case "-h":
                    printHelp();
                    System.exit(0);
                    break;

                default:
                    System.err.println("Unknown option: " + args[i]);
                    printHelp();
                    System.exit(1);
            }
        }

        // Print banner
        System.out.println(BANNER);

        // Create and start simulation engine
        SimulationEngine engine = new SimulationEngine();

        // Create and start server
        SimulatorServer server = new SimulatorServer(engine, port);

        try {
            server.start();
            engine.start();

            String url = "http://localhost:" + port;
            System.out.println();
            System.out.println("  Simulator running at: " + url);
            System.out.println();
            System.out.println("  Press Ctrl+C to quit");
            System.out.println();
            System.out.println("═".repeat(70));
            System.out.println();

            // Open browser
            if (openBrowser) {
                openBrowser(url);
            }

            // Add shutdown hook
            Runtime.getRuntime().addShutdownHook(new Thread(() -> {
                System.out.println("\nShutting down...");
                engine.stop();
                server.stop();
            }));

            // Keep main thread alive
            Thread.currentThread().join();

        } catch (Exception e) {
            System.err.println("Error starting simulator: " + e.getMessage());
            e.printStackTrace();
            System.exit(1);
        }
    }

    private static void openBrowser(String url) {
        try {
            if (Desktop.isDesktopSupported()) {
                Desktop desktop = Desktop.getDesktop();
                if (desktop.isSupported(Desktop.Action.BROWSE)) {
                    desktop.browse(new URI(url));
                    System.out.println("  Browser opened automatically");
                    return;
                }
            }

            // Fallback for different OS
            String os = System.getProperty("os.name").toLowerCase();
            ProcessBuilder pb;

            if (os.contains("mac")) {
                pb = new ProcessBuilder("open", url);
            } else if (os.contains("win")) {
                pb = new ProcessBuilder("cmd", "/c", "start", url);
            } else {
                pb = new ProcessBuilder("xdg-open", url);
            }

            pb.start();
            System.out.println("  Browser opened automatically");

        } catch (Exception e) {
            System.out.println("  Could not open browser automatically.");
            System.out.println("  Please open: " + url);
        }
    }

    private static void printHelp() {
        System.out.println("""
            Reefscape 2025 Simulator - Team 3164 Stealth Tigers

            Usage: java -jar reefscape-sim.jar [options]

            Options:
              --port, -p <port>   Server port (default: 8080)
              --no-browser        Don't auto-open browser
              --help, -h          Show this help

            Controls (in browser):
              WASD        - Drive robot
              Q/E         - Rotate
              1-4         - Set scoring level
              R           - Loading position
              Space       - Intake coral
              Shift       - Outtake/score
              X           - Toggle slow mode
              C           - Toggle field-relative
              G           - Reset gyro
              V           - Ski stop (lock wheels)
              Arrow Up/Down - Climber
              P           - Debug: pickup coral
              Escape      - Reset robot

            """);
    }
}
