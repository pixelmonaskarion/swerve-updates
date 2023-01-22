package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final int MOTOR_LEFT_MASTER_ID = 1; 
    public static final int MOTOR_LEFT_SLAVE_ID = 2;
    public static final int MOTOR_RIGHT_MASTER_ID = 3;
    public static final int MOTOR_RIGHT_SLAVE_ID = 4;

    public static final int JOYSTICK_PORT1 = 0;

    public static final int ANALOG_GYRO_PORT = 0;

    public static final double DEFAULT_DEADBAND = 0.02;
    public static final double WHEEL_RADIUS = Units.inchesToMeters(3); 
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * Math.PI;

    public static final double TRACK_WIDTH = Units.inchesToMeters(33); 
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);

    public enum JoystickScaling{
        QUADRATIC,
        CUBIC,
        LINEAR,
        SQRT,
        CBRT,
        LOGARITHMIC
     }
     
     public enum DriveStyle{
        CUSTOM_TANK,
        NORMAL_ARCADE,
        SATURATED_ARCADE,
        ARCADE_TANK
     }
}
