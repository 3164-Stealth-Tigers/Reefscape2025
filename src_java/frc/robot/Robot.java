package frc.robot;

/*
 * ========================================================================
 * ROBOT.JAVA - The Robot's Main Entry Point
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * This is the "main" class for the robot - it's where the robot code starts
 * and where the different robot modes (autonomous, teleop, etc.) are managed.
 *
 * Think of this as the "director" of a movie - it doesn't do the acting
 * (subsystems do that), but it tells everyone when to start and stop.
 *
 * HOW FRC ROBOT CODE WORKS:
 * -------------------------
 * The robot goes through different "modes" during a match:
 *
 *   1. DISABLED - Robot is on but can't move (before match starts)
 *   2. AUTONOMOUS - First 15 seconds, robot runs pre-programmed code
 *   3. TELEOP - Driver controls the robot (remaining time)
 *   4. TEST - Special mode for testing (not used in matches)
 *
 *   Match timeline:
 *   ┌──────────────────────────────────────────────────────────────┐
 *   │ DISABLED │     AUTO      │          TELEOP                   │
 *   │ waiting  │   15 seconds  │       2:15 remaining              │
 *   └──────────────────────────────────────────────────────────────┘
 *
 * THE ROBOT LOOP:
 * ---------------
 * Every 20 milliseconds (50 times per second), the robot:
 *   1. Reads sensors
 *   2. Runs the CommandScheduler (which runs all active commands)
 *   3. Updates motor outputs
 *
 * This is called the "periodic" loop. There are different periodic methods
 * for each mode (autonomousPeriodic, teleopPeriodic, etc.).
 *
 * TIMEDROBOT:
 * -----------
 * This class extends TimedRobot, which handles the timing loop for us.
 * We just override the methods we need (like robotInit, autonomousInit, etc.)
 * and WPILib calls them at the right times.
 *
 * THE COMMAND SCHEDULER:
 * ----------------------
 * We use "Command-based programming" where actions are encapsulated in
 * Command objects. The CommandScheduler:
 *   - Checks which buttons are pressed
 *   - Starts/stops commands based on triggers
 *   - Runs the execute() method of all active commands
 *   - Calls periodic() on all subsystems
 *
 * HOW TO MODIFY:
 * --------------
 * - Add initialization code: robotInit()
 * - Add autonomous startup code: autonomousInit()
 * - Add teleop startup code: teleopInit()
 * - Add code that runs every loop: robotPeriodic()
 *
 * IMPORTANT: Most robot code goes in RobotContainer, not here!
 * This file is mainly for lifecycle management (starting/stopping modes).
 *
 * QUICK REFERENCE:
 * ----------------
 * → Robot powers on:     robotInit() is called ONCE
 * → Every 20ms:          robotPeriodic() is called
 * → Auto starts:         autonomousInit() → autonomousPeriodic() runs
 * → Teleop starts:       teleopInit() → teleopPeriodic() runs
 * → Robot disabled:      disabledInit() → disabledPeriodic() runs
 *
 * ========================================================================
 */

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * ========================================================================
 * ROBOT CLASS - Main Robot Lifecycle Manager
 * ========================================================================
 *
 * The main robot class that WPILib runs automatically.
 * Extends TimedRobot to get the 20ms periodic loop.
 *
 * [ARCHITECTURE]
 * Robot.java is intentionally simple - it just manages mode transitions.
 * All the interesting robot code lives in RobotContainer and subsystems.
 *
 * [NAMING CONVENTION]
 * If you rename this class, you must also update build.gradle!
 * The class name must match the ROBOT_CLASS in build.gradle.
 */
public class Robot extends TimedRobot {

    // ========================================================================
    // STATE - Track autonomous command and robot container
    // ========================================================================

    /**
     * The autonomous command to run (selected from dashboard).
     * Set in autonomousInit(), cancelled in teleopInit().
     */
    private Command autonomousCommand;

    /**
     * The RobotContainer holds all subsystems and commands.
     * This is where most of the robot code lives.
     *
     * [WHY SEPARATE FROM ROBOT?]
     * Separation of concerns:
     *   - Robot.java = lifecycle management (when things happen)
     *   - RobotContainer = robot configuration (what things are)
     */
    private RobotContainer robotContainer;

    // ========================================================================
    // INITIALIZATION - Called once when robot powers on
    // ========================================================================

    /**
     * Called ONCE when the robot first boots up.
     *
     * [WHAT HAPPENS HERE]
     * 1. Start data logging (for post-match analysis)
     * 2. Create the RobotContainer (which creates all subsystems)
     *
     * [IMPORTANT]
     * This is NOT called at the start of each match!
     * It's only called when the robot powers on. If you need to do
     * something at the start of each match, use autonomousInit() or teleopInit().
     */
    @Override
    public void robotInit() {
        // Start recording data to log file
        // This saves NetworkTables values for post-match analysis
        // Logs are stored on the roboRIO and can be viewed with AdvantageScope
        DataLogManager.start();

        // Create the RobotContainer - this is where all the magic happens!
        // The constructor creates all subsystems, sets up button bindings,
        // and configures the autonomous chooser
        robotContainer = new RobotContainer();
    }

    // ========================================================================
    // PERIODIC - Called every 20ms regardless of mode
    // ========================================================================

    /**
     * Called every 20ms (50 times per second), in ALL modes.
     *
     * [THIS IS THE HEARTBEAT]
     * This method runs constantly whether disabled, in auto, or in teleop.
     * It's the main loop that keeps everything running.
     *
     * [WHAT HAPPENS HERE]
     * 1. CommandScheduler.run() - THE most important line!
     *    - Polls all controller buttons
     *    - Starts/stops commands based on triggers
     *    - Runs execute() on all active commands
     *    - Runs periodic() on all registered subsystems
     * 2. Log data to SmartDashboard for debugging
     *
     * [WARNING]
     * If you delete CommandScheduler.getInstance().run(), NOTHING will work!
     * No commands will run, no buttons will respond.
     */
    @Override
    public void robotPeriodic() {
        // THIS LINE IS CRITICAL - it runs the entire command framework!
        // Without this, no commands execute and no subsystem periodic() runs
        CommandScheduler.getInstance().run();

        // Log useful data to SmartDashboard for debugging
        robotContainer.logData();
    }

    // ========================================================================
    // DISABLED MODE - Robot is on but can't move
    // ========================================================================

    /**
     * Called once when the robot enters disabled mode.
     * This happens when you disable from the Driver Station,
     * or between autonomous and teleop during a match.
     */
    @Override
    public void disabledInit() {
        // Nothing to do here currently
        // You could add code to reset subsystems or stop motors
    }

    /**
     * Called every 20ms while disabled.
     * Use for diagnostic displays that should run even when disabled.
     */
    @Override
    public void disabledPeriodic() {
        // Nothing to do here currently
        // Commands cannot run while disabled (safety feature)
    }

    // ========================================================================
    // AUTONOMOUS MODE - Robot runs pre-programmed routine
    // ========================================================================

    /**
     * Called once when autonomous mode starts.
     *
     * [WHAT HAPPENS HERE]
     * 1. Get the autonomous command from RobotContainer
     *    (selected via the dashboard chooser)
     * 2. Schedule it to run
     *
     * [TIMING]
     * In FRC matches, autonomous is typically 15 seconds.
     * The command continues running until it finishes or teleop starts.
     */
    @Override
    public void autonomousInit() {
        // Get the autonomous command selected on the dashboard
        autonomousCommand = robotContainer.getAutonomousCommand();

        // If an auto was selected, start running it
        if (autonomousCommand != null) {
            autonomousCommand.schedule();  // Add to CommandScheduler's queue
        }
    }

    /**
     * Called every 20ms during autonomous.
     *
     * [NOTE]
     * We don't usually put code here because the CommandScheduler
     * handles running our autonomous command in robotPeriodic().
     */
    @Override
    public void autonomousPeriodic() {
        // CommandScheduler handles everything in robotPeriodic()
    }

    // ========================================================================
    // TELEOP MODE - Driver controls the robot
    // ========================================================================

    /**
     * Called once when teleop mode starts.
     *
     * [WHAT HAPPENS HERE]
     * Cancel any running autonomous command. This ensures the driver
     * has full control immediately when teleop starts.
     *
     * [WHY CANCEL AUTO?]
     * If auto is still running when teleop starts, the auto command
     * would conflict with driver controls. We cancel it to be safe.
     *
     * [ALTERNATIVE]
     * If you want auto to continue running (e.g., finish scoring),
     * you can comment out the cancel() call.
     */
    @Override
    public void teleopInit() {
        // Stop the autonomous command when teleop starts
        // This gives the driver full control immediately
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /**
     * Called every 20ms during teleop.
     *
     * [NOTE]
     * We don't usually put code here because the CommandScheduler
     * handles all teleop commands in robotPeriodic().
     */
    @Override
    public void teleopPeriodic() {
        // CommandScheduler handles everything in robotPeriodic()
    }

    // ========================================================================
    // TEST MODE - Special mode for testing (not used in matches)
    // ========================================================================

    /**
     * Called once when test mode starts.
     *
     * [WHAT IS TEST MODE?]
     * A special mode for testing subsystems outside of a match.
     * Enabled from the Driver Station by selecting "Test" mode.
     *
     * [WHAT HAPPENS HERE]
     * Cancel all running commands to start with a clean slate.
     */
    @Override
    public void testInit() {
        // Cancel all running commands for a clean test environment
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * Called every 20ms during test mode.
     *
     * [WHAT WE DO HERE]
     * Test the vision cameras to make sure they're working.
     * You can add other test code here as needed.
     */
    @Override
    public void testPeriodic() {
        // Test vision cameras
        robotContainer.getVision().testCameras();
    }

    // ========================================================================
    // SIMULATION MODE - Running in the simulator
    // ========================================================================

    /**
     * Called once when simulation starts.
     *
     * [WHAT IS SIMULATION?]
     * WPILib provides a robot simulator that lets you test code
     * without real hardware. Great for testing autonomous routines!
     */
    @Override
    public void simulationInit() {
        // Nothing to do here currently
        // You could initialize simulation-specific objects
    }

    /**
     * Called every 20ms during simulation.
     *
     * [WHAT YOU'D DO HERE]
     * Update physics simulation, simulate sensor values, etc.
     * Most simulation is handled by individual subsystems.
     */
    @Override
    public void simulationPeriodic() {
        // Nothing to do here currently
        // Subsystems handle their own simulation
    }

}  // End of Robot class
