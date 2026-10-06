package frc.robot.subsystems.tankdrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.motors.BaseMotor;
import frc.robot.subsystems.GenericSubsystem;
import frc.robot.utils.LimelightUtils;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.List;

import static java.util.Objects.*;

/**
 * Controls the drivetrain mechanisms.
 * @param <T> Motor type
 */
@SuppressWarnings("rawtypes")
public class TankDrive<T extends BaseMotor> extends GenericSubsystem {

    private final T leftMotor;
    private final T rightMotor;

    private final DifferentialDrive drive;
    private final DifferentialDriveKinematics kinematics;

    private final Field2d field = new Field2d();

    private Pose2d robotPose = new Pose2d(1.0, 2.0, new Rotation2d());

    // The X robot pose by meters
    private double xMeters = 0.0;
    // The Y robot pose by meters
    private double yMeters = 0.0;
    private Rotation2d headingRotation = Rotation2d.fromRadians(0.0);

    AHRS navx;
    DifferentialDriveOdometry odometry;
    private Rotation2d gyroAngle = headingRotation;

    /**
     * Controls the drivetrain mechanisms.
     * @param constants The constants for the drivetrain
     * @param pathplannerConfig The pathplanner config
     * @param frontLeft The front left motor
     * @param frontRight The front right motor
     * @param backLeft The back left motor (optional)
     * @param backRight The back right motor (optional)
     */
    public TankDrive(TankDriveConstants constants, RobotConfig pathplannerConfig, T frontLeft, T frontRight, @Nullable T backLeft, @Nullable T backRight) {
        super(constants.logPath());
        if (nonNull(backLeft) != nonNull(backRight)) {
            throw new IllegalArgumentException("Both back motors must be specified or neither.");
        }
        leftMotor = frontLeft;
        rightMotor = frontRight;
        if (nonNull(backLeft)) {
            follow(frontLeft, backLeft);
            follow(frontRight, backRight);
        }
        drive = new DifferentialDrive(leftMotor::setSpeed, rightMotor::setSpeed);
        SmartDashboard.putData("Field", field);

        odometry = new DifferentialDriveOdometry(headingRotation, leftMotor.getSpeed(), rightMotor.getSpeed());

        navx = new AHRS(AHRS.NavXComType.kMXP_SPI);
        navx.zeroYaw();

        kinematics = new DifferentialDriveKinematics(constants.trackWidthMeters());

//        AutoBuilder.configure(
////                LimelightUtils::getPose2d,
//                this::getRobotPose,
//                (Pose2d pose) -> {
//                    odometry.resetPosition(
//                            Rotation2d.fromDegrees(navx.getAngle()),
//                            leftMotor.getSpeed(),
//                            rightMotor.getSpeed(),
//                            pose
//                    );
//                    this.robotPose = pose;
//                },
//                this::getChassisSpeeds,
//                this::driveRobotRelative,
//                new PPLTVController(0.02),
//                pathplannerConfig,
//                () -> {
//                    var alliance = DriverStation.getAlliance();
//                    if (alliance.isPresent()) {
//                        return alliance.get() == DriverStation.Alliance.Red;
//                    }
//                    return false;
//                },
//                this
//        );

    }

    /**
     * Controls the drivetrain mechanisms.
     * <br>
     * Back motors are set to null.
     * @param constants The constants for the drivetrain
     * @param pathplannerConfig The pathplanner config
     * @param frontLeft The front left motor
     * @param frontRight The front right motor
     */
    public TankDrive(TankDriveConstants constants, RobotConfig pathplannerConfig, T frontLeft, T frontRight) {
        this(constants, pathplannerConfig, frontLeft, frontRight, null, null);
    }

    /**
     * Drives the robot using tank controls.
     * @param left The left side speed
     * @param right The right side speed
     */
    public void tankDrive(double left, double right) {
        drive.tankDrive(left, right, false);
    }

    /**
     * Drives the robot using arcade controls.
     * @param forward The forward/backword speed
     * @param right The right/left turn speed
     */
    public void arcadeDrive(double forward, double right) {
        drive.arcadeDrive(forward, right);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        Logger.recordOutput("Motors/Input/ChassisSpeeds", chassisSpeeds);

        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);

        wheelSpeeds.desaturate(5);

        Logger.recordOutput("Motors/Input/LeftFrontSpeed", wheelSpeeds.leftMetersPerSecond);
        Logger.recordOutput("Motors/Input/RightFrontSpeed", wheelSpeeds.rightMetersPerSecond);
        tankDrive(wheelSpeeds.leftMetersPerSecond / 5, wheelSpeeds.rightMetersPerSecond / 5);
    }

    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftMotor.getSpeed(), rightMotor.getSpeed()));
//        return new ChassisSpeeds(leftMotor.getSpeed(), rightMotor.getSpeed(), navx.getAngle());
    }

    @Override
    public void subsystemPeriodic() {
        Logger.recordOutput("Motors/Output/LeftFrontSpeed", leftMotor.getSpeed());
        Logger.recordOutput("Motors/Output/RightFrontSpeed", rightMotor.getSpeed());
        updatePose();
        gyroAngle = headingRotation;
        robotPose = odometry.update(gyroAngle, leftMotor.getSpeed(), rightMotor.getSpeed());
    }

    /**
     * Returns the robot's pose.
     * @return The robot's current pose.
     */
    public Pose2d getRobotPose() {
        return robotPose;
    }

    /**
     * Updates the robot's pose based on motor speeds and time step.
     */
    private void updatePose() {
        double lSpeed = leftMotor.getSpeed();
        double rSpeed = rightMotor.getSpeed();

        double dt = 0.02;
        double maxLinearSpeed = 3.0;
        double trackWidth = 0.6;

        double vL = lSpeed * maxLinearSpeed;
        double vR = rSpeed * maxLinearSpeed;

        double v = (vL + vR) / 2.0;
        double omega = (vR - vL) / trackWidth;

        headingRotation = headingRotation.plus(Rotation2d.fromRadians(omega * dt));

        xMeters += v * Math.cos(headingRotation.getRadians()) * dt;
        yMeters += v * Math.sin(headingRotation.getRadians()) * dt;

        robotPose = new Pose2d(xMeters, yMeters, headingRotation);

        field.setRobotPose(robotPose);
        Logger.recordOutput(getLogPath() + "/Pose", robotPose);
    }

    @Override
    public void simulationPeriodic() {
        Logger.recordOutput(getLogPath() + "/Angle", headingRotation);
    }

    // since every class that extends BaseMotor have matching types, we didn't specify them.
    // this makes the code more concise and readable, but giving some `unchecked` warnings.
    /**
     * Make the slave motor follow the master motor.
     * @param master The motor to follow.
     * @param slave The following motor.
     */
    @SuppressWarnings("unchecked")
    private void follow(T master, @NotNull T slave) {
        slave.follow(master, false);
    }
}
