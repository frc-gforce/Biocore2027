package frc.robot.motors;

import com.revrobotics.spark.SparkLowLevel.MotorType;

/**
 * Enumeration representing all the motors on the robot.
 */
public enum Motors {
    FRONT_LEFT(1, "Front Left Motor", MotorType.kBrushed, MotorBrand.SPARK_MAX, true, CanbusLoop.rio),
    FRONT_RIGHT(3, "Front Right Motor", MotorType.kBrushed, MotorBrand.SPARK_MAX, false, CanbusLoop.rio),
    BACK_RIGHT(8, "Back Right Motor", MotorType.kBrushed, MotorBrand.SPARK_MAX, false, CanbusLoop.rio),
    BACK_LEFT(6, "Back Left Motor", MotorType.kBrushed, MotorBrand.SPARK_MAX, true, CanbusLoop.rio),
    SHOOT(10, "Shoot Motor", MotorType.kBrushless, MotorBrand.TALON_FX, false, CanbusLoop.rio),
    FEED(23, "Feed Motor", MotorType.kBrushed, MotorBrand.SPARK_MAX, true, CanbusLoop.rio),
    BACK_RIGHT_INTAKE(60, "Back Right Intake Motor", MotorType.kBrushless, MotorBrand.TALON_FX, false, CanbusLoop.rio),
    BACK_LEFT_INTAKE(11, "Back Left Intake Motor", MotorType.kBrushless, MotorBrand.TALON_FX, false, CanbusLoop.rio),
    OPEN_INTAKE_RIGHT(10, "Open Intake Motor Right", MotorType.kBrushless, MotorBrand.TALON_FX, false, CanbusLoop.rio),
    OPEN_INTAKE_LEFT(9, "Open Intake Motor Left", MotorType.kBrushless, MotorBrand.TALON_FX, false, CanbusLoop.rio),
    CONVEYOR(1, "Conveyor Motor", MotorType.kBrushed, MotorBrand.SPARK_MAX, false, CanbusLoop.rio);

    public final int id;
    public final String name;
    public final MotorType type;
    public final MotorBrand brand;
    public final boolean inverted;
    public final CanbusLoop canbusLoop;

    Motors(int id, String name, MotorType type, MotorBrand brand, boolean inverted, CanbusLoop canbusLoop) {
        this.id = id;
        this.name = name;
        this.type = type;
        this.brand = brand;
        this.inverted = inverted ;
        this.canbusLoop = canbusLoop;
    }

    Motors(int id, String name, boolean inverted, MotorBrand brand, CanbusLoop canbusLoop) {
        this(id, name, null, brand, inverted, canbusLoop);
    }
}
