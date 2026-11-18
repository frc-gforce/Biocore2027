package frc.robot.subsystems.backintake;

import frc.robot.motors.BaseMotor;
import frc.robot.subsystems.GenericSubsystem;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("rawtypes")
public class BackIntake extends GenericSubsystem {

    private final BaseMotor leftIntakeMotor;
    private final BaseMotor rightIntakeMotor;
    private final BaseMotor openIntakeMotorLeft;
    private final BaseMotor openIntakeMotorRight;

    public BackIntake(BackIntakeConstants constants, BaseMotor leftIntakeMotor, BaseMotor rightIntakeMotor, BaseMotor openIntakeMotorLeft, BaseMotor openIntakeMotorRight) {
        super(constants.logPath());
        this.leftIntakeMotor = leftIntakeMotor;
        this.rightIntakeMotor = rightIntakeMotor;
        this.openIntakeMotorLeft = openIntakeMotorLeft;
        this.openIntakeMotorRight = openIntakeMotorRight;

        leftIntakeMotor.follow(rightIntakeMotor, true);
        openIntakeMotorLeft.follow(openIntakeMotorRight, true);
    }

    public void openIntake() {
        openIntakeMotorRight.setSpeed(0.2);
    }

    public void stopOpenIntake() {
        openIntakeMotorRight.setSpeed(0);
    }

    public void closeIntake() {
        openIntakeMotorRight.setSpeed(-0.5);
    }
    public void stopCloseIntake() {
        openIntakeMotorRight.setSpeed(0);
    }

    public void intake() {
        rightIntakeMotor.setSpeed(0.5);
    }

    public void stopIntake() {
        rightIntakeMotor.setSpeed(0);
    }

    public void emptying() {rightIntakeMotor.setSpeed(-0.5);}


    @Override
    protected void subsystemPeriodic() {
        Logger.recordOutput("OpenIntakeMotor", openIntakeMotorRight.getSpeed());
    }
}
