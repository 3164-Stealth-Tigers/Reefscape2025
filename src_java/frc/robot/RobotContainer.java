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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Controllers
    private final DriverActionSet driverJoystick;
    private final OperatorActionSet operatorJoystick;
    private final ScoringPositionsActionSet buttonBoard;

    // Subsystems
    private final SwerveDrive swerve;
    private final Elevator elevator;
    private final CoralArm coralArm;
    private final Claw claw;
    private final Climber climber;
    private final AlgaeArm algaeArm;
    private final AutoAlign autoAlign;
    private final Vision vision;

    // Superstructure coordinator
    private final Superstructure superstructure;

    // Autonomous chooser
    private final SendableChooser<Command> autoChooser;

    // State
    private boolean useAutomation = true;
    private int speedExponent = 2;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Silence joystick connection warnings
        DriverStation.silenceJoystickConnectionWarning(true);

        // Initialize controllers
        driverJoystick = new XboxDriver(0);
        operatorJoystick = new XboxOperator(1);
        buttonBoard = new ArcadeScoringPositions(2);

        // Initialize autonomous chooser
        autoChooser = new SendableChooser<>();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Initialize vision
        vision = new Vision();

        // Initialize swerve drive
        swerve = new SwerveDrive();

        // Set up teleop driving command
        Command teleopDriveCommand = swerve.teleopCommand(
            () -> applySpeedCurve(driverJoystick.forward()),
            () -> applySpeedCurve(driverJoystick.strafe()),
            () -> applySpeedCurve(driverJoystick.turn())
        );
        swerve.setDefaultCommand(teleopDriveCommand);
        swerve.resetYaw(Rotation2d.fromDegrees(180));
        SmartDashboard.putData("TeleOp Command", teleopDriveCommand);

        // Initialize elevator
        elevator = new Elevator();
        SmartDashboard.putData("Elevator", elevator);

        // Initialize coral arm
        coralArm = new CoralArm();
        SmartDashboard.putData("Coral Arm", coralArm);

        // Initialize claw
        claw = new Claw();
        claw.setDefaultCommand(claw.intakeCommand());
        SmartDashboard.putData("Claw", claw);

        // Initialize climber
        climber = new Climber();
        SmartDashboard.putData("Climber", climber);

        // Initialize algae arm
        algaeArm = new AlgaeArm();
        SmartDashboard.putData("Algae Arm", algaeArm);

        // Initialize auto-align
        autoAlign = new AutoAlign(swerve);

        // Initialize superstructure
        superstructure = new Superstructure(swerve, elevator, coralArm, climber, autoAlign);

        // Build autonomous routines
        buildForwardAuto();

        // Configure button bindings
        configureButtonBindings();

        // Setup automatic scoring
        new Trigger(() -> superstructure.readyToScore() && DriverStation.isTeleop() && useAutomation)
            .whileTrue(claw.outtakeCommand());
    }

    /**
     * Apply speed curve to joystick input.
     */
    private double applySpeedCurve(double input) {
        return Math.pow(Math.abs(input), speedExponent) * Math.signum(input);
    }

    /**
     * Configure button bindings for all controllers.
     */
    private void configureButtonBindings() {
        // Driver buttons
        driverJoystick.resetGyro().onTrue(swerve.resetGyroCommand());
        driverJoystick.toggleFieldRelative().onTrue(new InstantCommand(swerve::toggleFieldRelative));
        driverJoystick.skiStop().onTrue(
            SwerveCommands.skiStopCommand(swerve).until(driverJoystick::isMovementCommanded)
        );
        driverJoystick.toggleSpeed().onTrue(
            new InstantCommand(() -> speedExponent = (speedExponent == 1) ? 2 : 1)
        );

        // Operator automation toggle
        operatorJoystick.autoToggle().onTrue(
            new InstantCommand(() -> useAutomation = !useAutomation)
        );

        // Intake/outtake buttons
        operatorJoystick.intake().whileTrue(claw.intakeCommand());
        operatorJoystick.outtake().whileTrue(claw.outtakeCommand());

        // Elevator height buttons
        operatorJoystick.loadingLevel().onTrue(level0Command());
        operatorJoystick.level1().onTrue(level1Command());
        operatorJoystick.level2().onTrue(autoAlign.closeCommand(level2Command()));
        operatorJoystick.level3().onTrue(autoAlign.closeCommand(level3Command()));
        operatorJoystick.level4().onTrue(autoAlign.closeCommand(level4Command()));

        operatorJoystick.algaeArmExtended().onTrue(
            coralArm.setAngleCommand(Rotation2d.fromDegrees(90))
        );

        // Home elevator
        operatorJoystick.homeElevator().whileTrue(elevator.homeElevatorCommand());

        // Reef positions (A-L)
        configureReefPositions();

        // Climber buttons
        operatorJoystick.climberUp().whileTrue(climber.raiseRobotCommand());
        operatorJoystick.climberDown().whileTrue(climber.lowerRobotCommand());
    }

    /**
     * Configure reef position buttons.
     */
    private void configureReefPositions() {
        String[] positions = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"};
        Trigger[] triggers = {
            buttonBoard.reefA(), buttonBoard.reefB(), buttonBoard.reefC(), buttonBoard.reefD(),
            buttonBoard.reefE(), buttonBoard.reefF(), buttonBoard.reefG(), buttonBoard.reefH(),
            buttonBoard.reefI(), buttonBoard.reefJ(), buttonBoard.reefK(), buttonBoard.reefL()
        };

        for (int i = 0; i < positions.length; i++) {
            String position = positions[i];
            triggers[i].whileTrue(
                new AutoAlign.DriveToScoringPositionCommand(autoAlign, position)
            );
        }
    }

    // ==================== Autonomous Routines ====================

    /**
     * Build the simple forward auto routine.
     */
    private void buildForwardAuto() {
        Command auto = Commands.run(
            () -> swerve.drive(new Translation2d(1.5, 0), 0, false, false),
            swerve
        ).andThen(
            Commands.run(() -> swerve.drive(new Translation2d(), 0, false, false), swerve)
        ).withTimeout(2);

        autoChooser.setDefaultOption("Drive Forward", auto);
    }

    // ==================== Level Commands ====================

    /**
     * Command to set elevator and arm to level 0 (loading) position.
     */
    public Command level0Command() {
        return elevator.setHeightCommand(ScoringConstants.LOADING_HEIGHT)
            .alongWith(coralArm.setAngleCommand(ScoringConstants.LOADING_ANGLE))
            .withName("Level0Command");
    }

    /**
     * Command to set elevator and arm to level 1 position.
     */
    public Command level1Command() {
        return elevator.setHeightCommand(ScoringConstants.L1_HEIGHT)
            .alongWith(coralArm.setAngleCommand(ScoringConstants.L1_ANGLE))
            .withName("Level1Command");
    }

    /**
     * Command to set elevator and arm to level 2 position.
     */
    public Command level2Command() {
        return superstructure.setEndEffectorHeightCommand(ScoringConstants.L2_HEIGHT, ScoringConstants.L2_ANGLE)
            .withName("Level2Command");
    }

    /**
     * Command to set elevator and arm to level 3 position.
     */
    public Command level3Command() {
        return superstructure.setEndEffectorHeightCommand(ScoringConstants.L3_HEIGHT, ScoringConstants.L3_ANGLE)
            .withName("Level3Command");
    }

    /**
     * Command to set elevator and arm to level 4 position.
     */
    public Command level4Command() {
        return elevator.setHeightCommand(ScoringConstants.L4_HEIGHT)
            .alongWith(coralArm.setAngleCommand(ScoringConstants.L4_ANGLE))
            .withName("Level4Command");
    }

    /**
     * Command to back up off the reef.
     */
    public Command backupOffReef() {
        return new DriveDistanceCommand(swerve, -0.8, 0, 0.3);
    }

    // ==================== Accessors ====================

    /**
     * Get the autonomous command selected from the chooser.
     *
     * @return The selected autonomous command
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * Log data to SmartDashboard.
     */
    public void logData() {
        SmartDashboard.putBoolean("Slow Speed", speedExponent == 2);
        SmartDashboard.putBoolean("Use Automation", useAutomation);
    }

    /**
     * Get the vision subsystem for testing.
     */
    public Vision getVision() {
        return vision;
    }
}
