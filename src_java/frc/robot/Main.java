package frc.robot;
//something else
/*
 * ========================================================================
 * MAIN.JAVA - The Very First Code That Runs
 * ========================================================================
 *
 * WHAT THIS FILE DOES:
 * --------------------
 * This is the true entry point of the robot code - the first thing that
 * runs when the robot powers on. It simply tells WPILib "start running
 * the Robot class."
 *
 * [ANALOGY]
 * If Robot.java is the director of a movie, Main.java is the person
 * who yells "Action!" to start filming. It does one thing and then
 * gets out of the way.
 *
 * WHY IS IT SO SIMPLE?
 * --------------------
 * WPILib handles all the complex startup logic. We just need to point
 * it at our Robot class. All initialization should happen in Robot.java
 * and RobotContainer.java, NOT here.
 *
 * IMPORTANT WARNING:
 * ------------------
 * Do NOT add code to this file unless you REALLY know what you're doing!
 * - No static variables
 * - No initialization code
 * - No imports beyond what's needed
 *
 * If you need to initialize something at startup, put it in robotInit()
 * in Robot.java instead.
 *
 * ========================================================================
 */

import edu.wpi.first.wpilibj.RobotBase;

/**
 * The main entry point for robot code.
 *
 * [DO NOT MODIFY THIS FILE]
 * Unless you're changing the main robot class (very rare),
 * leave this file alone. All robot code goes elsewhere.
 */
public final class Main {

    /**
     * Private constructor prevents instantiation.
     * This class should never be instantiated - it only has a static main().
     */
    private Main() {}

    /**
     * The main() method - where Java programs begin execution.
     *
     * [WHAT THIS DOES]
     * Calls RobotBase.startRobot() with a reference to our Robot class.
     * WPILib then:
     *   1. Creates an instance of Robot
     *   2. Sets up the 20ms timing loop
     *   3. Starts calling robotInit(), robotPeriodic(), etc.
     *
     * [THE Robot::new SYNTAX]
     * This is a "method reference" in Java. It's shorthand for:
     *   () -> new Robot()
     *
     * It tells WPILib how to create a new Robot instance.
     *
     * @param args Command-line arguments (not used in FRC)
     */
    public static void main(String... args) {
        // Start the robot! WPILib takes it from here.
        RobotBase.startRobot(Robot::new);
    }

}  // End of Main class
