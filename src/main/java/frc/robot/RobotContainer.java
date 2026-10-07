// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.*;
import frc.robot.controller.FlightSimulatorController;
import frc.robot.motors.*;
import frc.robot.motors.TalonFXMotor;
import frc.robot.subsystems.backintake.BackIntake;
import frc.robot.subsystems.feed.Feed;
import frc.robot.subsystems.swervedrive.SwerveDriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intakexshooter.IntakeXShooter;
import frc.robot.utils.*;
import swervelib.SwerveInputStream;

import java.util.function.BooleanSupplier;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    //    private final TankDrive<SparkBaseMotor> tankDrive;
    private final IntakeXShooter intakeXShooter;
    private final Feed feed;
    private final SwerveDriveSubsystem swerveDrive;
    private final BackIntake backIntake;


    private final Joystick driver = new Joystick(OperatorConstants.DRIVER_CONTROLLER_PORT);
    // The robot's subsystems and commands are defined here...
    private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

    // Replace with CommandPS4Controller or CommandJoystick if needed
//    private final GenericCommandController driverController =
//            new XboxCommandController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final FlightSimulatorController driverController = new FlightSimulatorController(OperatorConstants.DRIVER_CONTROLLER_PORT);

    private final SendableChooser<Command> driverChooser = new SendableChooser<>();
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

//    private final SparkMaxMotor frontLeftMotor;
//    private final SparkMaxMotor frontRightMotor;
//    private final SparkMaxMotor backLeftMotor;
//    private final SparkMaxMotor backRightMotor;

    private final TalonFXMotor shootMotor;
    private final SparkMaxMotor feedMotor;
    private final SparkMaxMotor conveyorMotor;

    private final TalonFXMotor rightIntakeMotor;
    private final TalonFXMotor leftIntakeMotor;
    private final SparkMaxMotor openIntakeMotorLeft;
    private final SparkMaxMotor openIntakeMotorRight;


    private final SwerveInputStream driveAngularInput;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        //<editor-fold desc="Motors creation">
//        frontLeftMotor = MotorFactory.createSparkMotor(Motors.FRONT_LEFT);
//        frontRightMotor = MotorFactory.createSparkMotor(Motors.FRONT_RIGHT);
//        backLeftMotor = MotorFactory.createSparkMotor(Motors.BACK_LEFT);
//        backRightMotor = MotorFactory.createSparkMotor(Motors.BACK_RIGHT);

        shootMotor = MotorFactory.createTalonFXMotor(Motors.SHOOT);
        shootMotor.setPID(new PID(
//                0.11,
//                0,
//                0,
//                0.2199,
//                0.1178,
//                0,
//                0
                0.45,
                0,
                0,
                0,
                0.12,
                0,
                0
        ));
        feedMotor = MotorFactory.createSparkMotor(Motors.FEED);
        conveyorMotor = MotorFactory.createSparkMotor(Motors.CONVEYOR);


        rightIntakeMotor = MotorFactory.createTalonFXMotor(Motors.BACK_RIGHT_INTAKE);
        leftIntakeMotor = MotorFactory.createTalonFXMotor(Motors.BACK_LEFT_INTAKE);
        openIntakeMotorLeft = MotorFactory.createSparkMotor(Motors.OPEN_INTAKE_LEFT);
        openIntakeMotorRight = MotorFactory.createSparkMotor(Motors.OPEN_INTAKE_RIGHT);

        openIntakeMotorLeft.setBreak(true);
        openIntakeMotorRight.setBreak(true);

        //</editor-fold>


        //<editor-fold desc="Default motors configs">
        //TODO put this in a more convenient place
//        frontLeftMotor.setBreak(false);
//        frontRightMotor.setBreak(false);
        shootMotor.setBreak(false);
        feedMotor.setBreak(false);
        //</editor-fold>

        //<editor-fold desc="Subsystems creation">
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
//        tankDrive = new TankDrive<>(
//                DrivetrainConstantsFactory.createDrivetrainConstants(),
//                config,
//                frontLeftMotor,
//                frontRightMotor,
//                backLeftMotor,
//                backRightMotor
//        );
        intakeXShooter = new IntakeXShooter(
                IntakeXShooterConstantsFactory.createIntakeXShooterConstants(),
                shootMotor,
                0.5,
                0.05
        );
        feed = new Feed(FeedConstantsFactory.createConstants(), feedMotor, conveyorMotor);

        backIntake = new BackIntake(
                BackIntakeConstantsFactory.createConstants(),
                leftIntakeMotor,
                rightIntakeMotor,
                openIntakeMotorLeft,
                openIntakeMotorRight
        );
        //</editor-fold>

        swerveDrive = new SwerveDriveSubsystem();
        driveAngularInput = SwerveInputStream.of(swerveDrive.getSwerveDrive(),
                        () -> driverController.getJoystickX() * -1,
                        () -> driverController.getJoystickY() * -1)
                .withControllerRotationAxis(() -> driverController.getRotate() * -1)
                .deadband(0.1)
//                .scaleTranslation(0.8)
                .allianceRelativeControl(false);
        swerveDrive.setDefaultCommand(swerveDrive.driveFieldOriented(driveAngularInput));
//        swerveDrive.setDefaultCommand(swerveDrive.driveRobotRelative(driveAngularInput));
//        swerveDrive.setDefaultCommand(swerveDrive.driveCommand(
//                () -> -MathUtil.applyDeadband(driverController.getLeftY(), 0.1), // קדימה/אחורה
//                () -> -MathUtil.applyDeadband(driverController.getLeftX(), 0.1), // ימינה/שמאלה
//                () -> -MathUtil.applyDeadband(driverController.getRightX(), 0.1), // סיבוב
//                () -> -MathUtil.applyDeadband(driverController.getRightY(), 0.1) // סיבוב
//        ));

//        driverChooser.setDefaultOption("Arcade", new DriveArcadeCommand(tankDrive, driverController));
//        driverChooser.addOption("Tank", new DriveTankCommand(tankDrive, driverController));
//        SmartDashboard.putData("Drive Mode", driverChooser);
//        drivetrain.setDefaultCommand(driverChooser.getSelected());

        loadAutos();
        // Configure the trigger bindings
        configureBindings();
    }

    private void loadAutos() {
        NamedCommands.registerCommand("OpenIntake", openIntakeCommand().withTimeout(4));
        NamedCommands.registerCommand("Shoot", shootCommand());

        autoChooser = AutoBuilder.buildAutoChooser();

        Command autonomousShootCommand = GenericSubsystemCommandFactory.getAsSubsystemsCommand(
//                new AutonomousShootCommand(intakeXShooter, feed, 10),
                aimCommand().onlyIf(() -> FieldUtils.isInAllianceSide(swerveDrive.getPose())),
                "Auto Shoot Command");
        autoChooser.addOption("Bottom Ramp", Autos.bottomRampAuto(autonomousShootCommand, backIntakeCommand(), swerveDrive.stop()));
        autoChooser.addOption("Bottom Trench", Autos.bottomTrenchAuto(autonomousShootCommand, backIntakeCommand()));
        autoChooser.addOption("Upper Ramp", Autos.upperRampAuto(autonomousShootCommand, backIntakeCommand()));
        autoChooser.addOption("Upper Trench", Autos.upperTrenchAuto(autonomousShootCommand, backIntakeCommand()));
        autoChooser.addOption("Middle", Autos.bottomTrenchToMiddle(autonomousShootCommand, backIntakeCommand(), openIntakeCommand()));
//        autoChooser.addOption("Test", Autos.testAuto());

        SmartDashboard.putData("Auto", autoChooser);
    }


    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link CommandPS4Controller
     * PS4} controllers or {@link CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        new Trigger(exampleSubsystem::exampleCondition)
                .onTrue(new ExampleCommand(exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
        // cancelling on release.
//        driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
//        driverController.faceLeft().onTrue(new InstantCommand(this::switchBreak));
//        driverController.faceLeft().whileTrue(backIntakeCommand);
//        driverController.faceRight().whileTrue(new SequentialCommandGroup(aimCommand.onlyIf(() -> FieldUtils.isInAllianceSide(swerveDrive.getPose())), shootCommand));
//        driverController.faceUp().whileTrue(closeIntakeCommand);
//        driverController.faceDown().whileTrue(openIntakeCommand);

        // Shoot
        new Trigger(() -> driverController.getThrottle() * -1 > OperatorConstants.throttleThreshold).whileTrue(
//                new SequentialCommandGroup(aimCommand().onlyIf(() -> FieldUtils.isInAllianceSide(swerveDrive.getPose())), shootCommand())
                shootOnTheMoveCommand()
        );
        // Intake
        new Trigger(() -> driverController.getThrottle() * -1 < OperatorConstants.throttleThreshold * -1).whileTrue(
                backIntakeCommand()
        );
        // Shoot + Intake
        driverController.f1().whileTrue(
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                aimCommand().onlyIf(() -> FieldUtils.isInAllianceSide(swerveDrive.getPose())),
                                shootCommand()
                        ),
                        backIntakeCommand()
                )
        );

        driverController.f2().whileTrue(emptyCommand());
        driverController.x().whileTrue(closeIntakeCommand());
        driverController.a().whileTrue(openIntakeCommand());

        //sets the gyro to zero
        //driverController.y().onTrue(new InstantCommand(swerveDrive::zeroGyro));
        //TODO make to gyro go to zero
    }

//    private void switchBreak() {
//        boolean breakState = frontLeftMotor.getBreak();
//        frontLeftMotor.setBreak(!breakState);
//        frontRightMotor.setBreak(!breakState);
//        backLeftMotor.setBreak(!breakState);
//        backRightMotor.setBreak(!breakState);
//    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
//        return Autos.exampleAuto(exampleSubsystem);
        return autoChooser.getSelected();
    }

    public Command getDriveCommand() {
//        return swerveDrive.getDefaultCommand();
        return driverChooser.getSelected();
    }

    public void resetOdometry() {
        swerveDrive.resetOdometry(LimelightUtils.getPose2d());
    }

    //<editor-fold desc="Commands creation">
    private Command shootCommand() {
        return GenericSubsystemCommandFactory.getAsSubsystemsCommand(
                new ShootCommand(intakeXShooter, feed, () -> swerveDrive.getPose().getTranslation(), FieldUtils::getHubPose),
                "Shoot Command"
        );
    }

    private Command shootOnTheMoveCommand() {
        return GenericSubsystemCommandFactory.getAsSubsystemsCommand(
                new ShootOnTheMoveCommand(swerveDrive, intakeXShooter, feed, driveAngularInput),
                "Shoot on the move Command"
        );
    }

    private Command intakeCommand() {
        return GenericSubsystemCommandFactory.getAsSubsystemsCommand(
                new IntakeCommand(intakeXShooter, feed),
                "Intake Command"
        );
    }

    private Command backIntakeCommand() {
        return GenericSubsystemCommandFactory.getAsSubsystemsCommand(
                new BackIntakeCommand(backIntake),
                "Back Intake Command"
        );
    }

    private Command openIntakeCommand() {
        return GenericSubsystemCommandFactory.getAsSubsystemsCommand(
                new OpenIntakeCommand(backIntake, feed),
                "Open Intake Command"
        );
    }

    private Command closeIntakeCommand() {
        return GenericSubsystemCommandFactory.getAsSubsystemsCommand(
                new CloseIntakeCommand(backIntake),
                "Close Intake Command"
        );
    }

    private Command aimCommand() {
        return GenericSubsystemCommandFactory.getAsSubsystemsCommand(
                new AimAtHubCommand(swerveDrive),
                "Aim Command"
        );
    }

    private Command emptyCommand() {
        return GenericSubsystemCommandFactory.getAsSubsystemsCommand(
                new EmptyingContainer(feed, backIntake),
                "Empty Container Command"
        );
    }
    //</editor-fold>
}
