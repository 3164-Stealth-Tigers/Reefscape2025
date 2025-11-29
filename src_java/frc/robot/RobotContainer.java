package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ScoringConstants;
import frc.robot.OI.ArcadeScoringPositions;
import frc.robot.OI.DriverActionSet;
import frc.robot.OI.OperatorActionSet;
import frc.robot.OI.ScoringPositionsActionSet;
import frc.robot.OI.XboxDriver;
import frc.robot.OI.XboxOperator;
import frc.robot.commands.SwerveCommands;
import frc.robot.commands.SwerveCommands.DriveDistanceCommand;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.AutoAlign;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swerve.SwerveDrive;

/*
 * ============================================================================
 * ROBOTCONTAINER.JAVA - The Robot's Command Center
 * ============================================================================
 *
 * WHAT THIS FILE DOES:
 * This is where everything comes together! RobotContainer:
 * 1. Creates all the subsystems (elevator, arm, drivetrain, etc.)
 * 2. Maps controller buttons to robot actions
 * 3. Sets up autonomous routines
 *
 * Think of it like the main control panel that wires everything together.
 *
 * ============================================================================
 * QUICK REFERENCE - Common Tasks
 * ============================================================================
 *
 * ADD A NEW BUTTON BINDING:
 *   → Go to configureButtonBindings() method
 *   → Add: joystick.buttonName().onTrue(yourCommand);
 *
 * CHANGE WHICH CONTROLLER IS USED:
 *   → Find the controller initialization in the constructor
 *   → Change XboxDriver to PS4Driver, etc.
 *
 * ADD A NEW AUTONOMOUS ROUTINE:
 *   → Create a new build___Auto() method
 *   → Add it to the autoChooser
 *
 * CHANGE LEVEL HEIGHTS/ANGLES:
 *   → Don't change here! Go to Constants.java > ScoringConstants
 *
 * ============================================================================
 * HOW BUTTON BINDINGS WORK
 * ============================================================================
 *
 * Button bindings connect controller inputs to robot commands:
 *
 *   button.onTrue(command)    → Run command ONCE when button is pressed
 *   button.whileTrue(command) → Run command WHILE button is held
 *   button.toggleOnTrue()     → Toggle command on/off each press
 *
 * Example:
 *   driverJoystick.resetGyro().onTrue(swerve.resetGyroCommand());
 *   ↑ controller ↑ button     ↑ action      ↑ what happens
 *
 * ============================================================================
 */
public class RobotContainer {

    // ========================================================================
    // CONTROLLERS
    // ========================================================================
    // These interfaces define what inputs we need from each controller.
    // The actual controller type (Xbox, PS4, etc.) is set in the constructor.
    // ========================================================================
    private final DriverActionSet driverJoystick;       // Port 0: Moves the robot
    private final OperatorActionSet operatorJoystick;   // Port 1: Controls mechanisms
    private final ScoringPositionsActionSet buttonBoard; // Port 2: Quick reef positions

    // ========================================================================
    // SUBSYSTEMS - The physical parts of the robot
    // ========================================================================
    // Each subsystem controls one mechanism. They're created here and used
    // throughout this file to make commands and button bindings.
    // ========================================================================
    private final SwerveDrive swerve;      // Drivetrain - moves the robot around
    private final Elevator elevator;       // Lifts game pieces up and down
    private final CoralArm coralArm;       // Rotates to position pieces for scoring
    private final Claw claw;               // Grabs and releases game pieces
    private final Climber climber;         // Lifts robot during end-game
    private final AlgaeArm algaeArm;       // Secondary arm for algae
    private final AutoAlign autoAlign;     // Automatic positioning for scoring
    private final Vision vision;           // Camera system for tracking targets

    // ========================================================================
    // SUPERSTRUCTURE - Coordinates multiple subsystems
    // ========================================================================
    private final Superstructure superstructure;

    // ========================================================================
    // AUTONOMOUS CHOOSER
    // ========================================================================
    // Lets drivers select which auto routine to run from SmartDashboard
    private final SendableChooser<Command> autoChooser;

    // ========================================================================
    // STATE VARIABLES
    // ========================================================================
    private boolean useAutomation = true;  // Auto-score when ready?
    private int speedExponent = 2;         // Joystick curve (1=linear, 2=squared)

    // ========================================================================
    // CONSTRUCTOR - Sets up everything when robot starts
    // ========================================================================

    /**
     * Creates the RobotContainer. This runs once when the robot powers on.
     *
     * ORDER MATTERS! We create things in this order:
     * 1. Silence warnings (so driver station doesn't spam errors)
     * 2. Create controllers
     * 3. Create auto chooser
     * 4. Create subsystems
     * 5. Set up driving command
     * 6. Build auto routines
     * 7. Configure button bindings
     */
    public RobotContainer() {
        // Prevents "joystick not connected" warnings during setup
        DriverStation.silenceJoystickConnectionWarning(true);

        // --------------------------------------------------------------------
        // STEP 1: CREATE CONTROLLERS
        // --------------------------------------------------------------------
        // Change these to use different controller types:
        // - XboxDriver, PS4Driver, T16000MDriver (for driver)
        // - XboxOperator (for operator)
        // - ArcadeScoringPositions, PS4ScoringPositions, KeyboardScoringPositions
        // --------------------------------------------------------------------
        driverJoystick = new XboxDriver(0);           // Driver uses Xbox on port 0
        operatorJoystick = new XboxOperator(1);       // Operator uses Xbox on port 1
        buttonBoard = new ArcadeScoringPositions(2);  // Button board on port 2

        // --------------------------------------------------------------------
        // STEP 2: CREATE AUTO CHOOSER
        // --------------------------------------------------------------------
        // This dropdown appears in SmartDashboard for selecting auto modes
        autoChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // --------------------------------------------------------------------
        // STEP 3: CREATE SUBSYSTEMS
        // --------------------------------------------------------------------
        // Each subsystem manages one part of the robot hardware
        vision = new Vision();
        swerve = new SwerveDrive();
        elevator = new Elevator();
        coralArm = new CoralArm();
        claw = new Claw();
        climber = new Climber();
        algaeArm = new AlgaeArm();
        autoAlign = new AutoAlign(swerve);
        superstructure = new Superstructure(swerve, elevator, coralArm, climber, autoAlign);

        // --------------------------------------------------------------------
        // STEP 4: SET UP DRIVING (runs continuously in teleop)
        // --------------------------------------------------------------------
        // This command reads joystick inputs and drives the robot
        Command teleopDriveCommand = swerve.teleopCommand(
            () -> applySpeedCurve(driverJoystick.forward()),  // Forward/backward
            () -> applySpeedCurve(driverJoystick.strafe()),   // Left/right
            () -> applySpeedCurve(driverJoystick.turn())      // Rotation
        );
        swerve.setDefaultCommand(teleopDriveCommand);

        // Start facing away from driver station (180 degrees)
        swerve.resetYaw(Rotation2d.fromDegrees(180));

        // Put subsystems on SmartDashboard for debugging
        SmartDashboard.putData("TeleOp Command", teleopDriveCommand);
        SmartDashboard.putData("Elevator", elevator);
        SmartDashboard.putData("Coral Arm", coralArm);
        SmartDashboard.putData("Claw", claw);
        SmartDashboard.putData("Climber", climber);
        SmartDashboard.putData("Algae Arm", algaeArm);

        // Claw runs intake by default (always trying to grab pieces)
        claw.setDefaultCommand(claw.intakeCommand());

        // --------------------------------------------------------------------
        // STEP 5: BUILD AUTO ROUTINES
        // --------------------------------------------------------------------
        buildForwardAuto();
        // Add more auto routines here:
        // buildScoreAndMoveAuto();
        // buildTwoPieceAuto();

        // --------------------------------------------------------------------
        // STEP 6: CONFIGURE BUTTON BINDINGS
        // --------------------------------------------------------------------
        configureButtonBindings();

        // --------------------------------------------------------------------
        // STEP 7: AUTOMATIC SCORING TRIGGER
        // --------------------------------------------------------------------
        // When everything is ready and automation is enabled, auto-score!
        new Trigger(() -> superstructure.readyToScore() && DriverStation.isTeleop() && useAutomation)
            .whileTrue(claw.outtakeCommand());
    }

    // ========================================================================
    // HELPER METHODS
    // ========================================================================

    /**
     * Applies a curve to joystick input for better control feel.
     *
     * WHY USE THIS?
     * Raw joystick input is linear: 50% stick = 50% speed.
     * With squared input: 50% stick = 25% speed (easier precision at low speeds)
     *
     * @param input Raw joystick value (-1 to 1)
     * @return Curved value (-1 to 1)
     */
    private double applySpeedCurve(double input) {
        // Math.pow gives us exponential curve
        // Math.signum keeps the positive/negative sign
        return Math.pow(Math.abs(input), speedExponent) * Math.signum(input);
    }

    // ========================================================================
    // BUTTON BINDINGS
    // ========================================================================

    /**
     * Maps controller buttons to robot actions.
     *
     * HOW TO ADD A NEW BINDING:
     * 1. Decide which controller (driverJoystick, operatorJoystick, buttonBoard)
     * 2. Pick a button method (see OI.java for available buttons)
     * 3. Choose a trigger type:
     *    - onTrue()     = run once when pressed
     *    - whileTrue()  = run while held, stop when released
     *    - toggleOnTrue() = toggle on/off each press
     * 4. Provide the command to run
     *
     * EXAMPLE:
     *   operatorJoystick.level3().onTrue(level3Command());
     *   // When operator presses B button, go to level 3 position
     */
    private void configureButtonBindings() {

        // ====================================================================
        // DRIVER CONTROLS - Moving the robot
        // ====================================================================

        // Reset gyro to 180° (robot facing away from driver)
        // Press START button to recalibrate "forward" direction
        driverJoystick.resetGyro().onTrue(swerve.resetGyroCommand());

        // Toggle between field-relative and robot-relative driving
        // Field-relative: push forward = robot goes toward opposing alliance
        // Robot-relative: push forward = robot goes where it's facing
        driverJoystick.toggleFieldRelative().onTrue(
            new InstantCommand(swerve::toggleFieldRelative)
        );

        // Ski stop: lock wheels in X pattern to resist being pushed
        // Hold Y button to engage, releases when driver starts moving
        driverJoystick.skiStop().onTrue(
            SwerveCommands.skiStopCommand(swerve).until(driverJoystick::isMovementCommanded)
        );

        // Toggle between squared input (precise) and linear input (fast)
        // Press X to switch between modes
        driverJoystick.toggleSpeed().onTrue(
            new InstantCommand(() -> speedExponent = (speedExponent == 1) ? 2 : 1)
        );

        // ====================================================================
        // OPERATOR CONTROLS - Mechanisms
        // ====================================================================

        // Toggle automatic scoring on/off
        operatorJoystick.autoToggle().onTrue(
            new InstantCommand(() -> useAutomation = !useAutomation)
        );

        // --------------------------------------------------------------------
        // INTAKE/OUTTAKE - Grabbing and releasing pieces
        // --------------------------------------------------------------------
        operatorJoystick.intake().whileTrue(claw.intakeCommand());   // Hold to intake
        operatorJoystick.outtake().whileTrue(claw.outtakeCommand()); // Hold to release

        // --------------------------------------------------------------------
        // SCORING LEVELS - Preset positions for elevator and arm
        // --------------------------------------------------------------------
        // Each button moves elevator and arm to the right height/angle

        // Right Trigger → Loading position (Level 0)
        operatorJoystick.loadingLevel().onTrue(level0Command());

        // A Button → Level 1 (lowest scoring)
        operatorJoystick.level1().onTrue(level1Command());

        // X Button → Level 2 (waits until close to reef)
        operatorJoystick.level2().onTrue(autoAlign.closeCommand(level2Command()));

        // B Button → Level 3 (waits until close to reef)
        operatorJoystick.level3().onTrue(autoAlign.closeCommand(level3Command()));

        // Y Button → Level 4 (highest scoring, waits until close to reef)
        operatorJoystick.level4().onTrue(autoAlign.closeCommand(level4Command()));

        // POV Left → Extend coral arm to 90° (for special maneuvers)
        operatorJoystick.algaeArmExtended().onTrue(
            coralArm.setAngleCommand(Rotation2d.fromDegrees(90))
        );

        // --------------------------------------------------------------------
        // ELEVATOR HOMING - Recalibrates elevator position
        // --------------------------------------------------------------------
        // Hold START to slowly lower elevator until limit switch triggers
        // This resets the "zero" position if the elevator drifts
        operatorJoystick.homeElevator().whileTrue(elevator.homeElevatorCommand());

        // --------------------------------------------------------------------
        // CLIMBER - End-game robot lifting
        // --------------------------------------------------------------------
        operatorJoystick.climberUp().whileTrue(climber.raiseRobotCommand());
        operatorJoystick.climberDown().whileTrue(climber.lowerRobotCommand());

        // --------------------------------------------------------------------
        // BUTTON BOARD - Quick reef position selection
        // --------------------------------------------------------------------
        // Arcade buttons for instant positioning at reef locations A-L
        configureReefPositions();
    }

    /**
     * Sets up buttons for each reef scoring position (A through L).
     *
     * The reef has 12 scoring positions arranged in a hexagon:
     *
     *         K ---- L
     *        /        \
     *       J          A
     *       |   REEF   |
     *       I          B
     *        \        /
     *         H ---- C
     *        /        \
     *       G          D
     *        \        /
     *         F ---- E
     *
     * Pressing a button drives the robot to that position automatically.
     */
    private void configureReefPositions() {
        // Array of position labels
        String[] positions = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"};

        // Array of corresponding buttons from the button board
        Trigger[] triggers = {
            buttonBoard.reefA(), buttonBoard.reefB(), buttonBoard.reefC(), buttonBoard.reefD(),
            buttonBoard.reefE(), buttonBoard.reefF(), buttonBoard.reefG(), buttonBoard.reefH(),
            buttonBoard.reefI(), buttonBoard.reefJ(), buttonBoard.reefK(), buttonBoard.reefL()
        };

        // Connect each button to its auto-drive command
        for (int i = 0; i < positions.length; i++) {
            String position = positions[i];
            triggers[i].whileTrue(
                new AutoAlign.DriveToScoringPositionCommand(autoAlign, position)
            );
        }
    }

    // ========================================================================
    // AUTONOMOUS ROUTINES
    // ========================================================================
    // Each method creates an auto routine and adds it to the chooser.
    // Drivers select which one to run from SmartDashboard before the match.
    // ========================================================================

    /**
     * Simple auto that just drives forward for 2 seconds.
     * Good for testing or when you just need to leave the starting zone.
     */
    private void buildForwardAuto() {
        Command auto = Commands.run(
            // Drive forward at 1.5 m/s, no strafe, no rotation
            () -> swerve.drive(new Translation2d(1.5, 0), 0, false, false),
            swerve
        ).andThen(
            // Stop after driving
            Commands.run(() -> swerve.drive(new Translation2d(), 0, false, false), swerve)
        ).withTimeout(2);  // Total runtime: 2 seconds

        // Add to the auto chooser
        autoChooser.setDefaultOption("Drive Forward", auto);
    }

    /*
     * HOW TO ADD A NEW AUTO ROUTINE:
     * --------------------------------
     *
     * 1. Create a new method (copy the pattern below):
     *
     *    private void buildMyAuto() {
     *        Command auto = Commands.sequence(
     *            // Step 1: Do something
     *            firstCommand(),
     *            // Step 2: Do something else
     *            secondCommand()
     *        );
     *        autoChooser.addOption("My Auto Name", auto);
     *    }
     *
     * 2. Call your method in the constructor (after buildForwardAuto())
     *
     * 3. The new auto will appear in the SmartDashboard dropdown!
     *
     * COMMAND COMPOSITION TIPS:
     * - Commands.sequence(): Do things one after another
     * - Commands.parallel(): Do things at the same time
     * - Commands.race(): Run until ANY command finishes
     * - command.withTimeout(2.0): Add a time limit (seconds)
     */

    // ========================================================================
    // LEVEL COMMANDS
    // ========================================================================
    // These commands move the elevator and arm to preset positions.
    // Heights and angles are defined in Constants.java > ScoringConstants
    // ========================================================================

    /**
     * Level 0 (Loading): Pickup position for grabbing pieces.
     * Elevator low, arm horizontal.
     */
    public Command level0Command() {
        return elevator.setHeightCommand(ScoringConstants.LOADING_HEIGHT)
            .alongWith(coralArm.setAngleCommand(ScoringConstants.LOADING_ANGLE))
            .withName("Level0Command");
    }

    /**
     * Level 1: Lowest scoring position (trough).
     */
    public Command level1Command() {
        return elevator.setHeightCommand(ScoringConstants.L1_HEIGHT)
            .alongWith(coralArm.setAngleCommand(ScoringConstants.L1_ANGLE))
            .withName("Level1Command");
    }

    /**
     * Level 2: Second level scoring.
     * Uses superstructure to coordinate elevator and arm together.
     */
    public Command level2Command() {
        return superstructure.setEndEffectorHeightCommand(ScoringConstants.L2_HEIGHT, ScoringConstants.L2_ANGLE)
            .withName("Level2Command");
    }

    /**
     * Level 3: Third level scoring.
     */
    public Command level3Command() {
        return superstructure.setEndEffectorHeightCommand(ScoringConstants.L3_HEIGHT, ScoringConstants.L3_ANGLE)
            .withName("Level3Command");
    }

    /**
     * Level 4: Highest scoring position (most points!).
     */
    public Command level4Command() {
        return elevator.setHeightCommand(ScoringConstants.L4_HEIGHT)
            .alongWith(coralArm.setAngleCommand(ScoringConstants.L4_ANGLE))
            .withName("Level4Command");
    }

    /**
     * Backs the robot away from the reef after scoring.
     * Drives backward 0.8 meters over 0.3 seconds.
     */
    public Command backupOffReef() {
        return new DriveDistanceCommand(swerve, -0.8, 0, 0.3);
    }

    // ========================================================================
    // PUBLIC ACCESSORS
    // ========================================================================
    // Methods that other classes can call to get information or subsystems
    // ========================================================================

    /**
     * Gets the autonomous command selected from the dashboard.
     * Called by Robot.java when autonomous period starts.
     *
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Logs telemetry data to SmartDashboard.
     * Called every loop by Robot.java.
     */
    public void logData() {
        SmartDashboard.putBoolean("Slow Speed", speedExponent == 2);
        SmartDashboard.putBoolean("Use Automation", useAutomation);
    }

    /**
     * Gets the vision subsystem (for testing).
     *
     * @return The vision subsystem
     */
    public Vision getVision() {
        return vision;
    }
}
