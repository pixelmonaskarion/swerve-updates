package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;



public final class Constants {
   public static final class RobotMap{
      
    public static final int MOTOR_LEFT_MASTER_ID = 2; //motor port values are the same as encoder port values
    public static final int MOTOR_LEFT_SLAVE_ID = 1;
    public static final int MOTOR_RIGHT_MASTER_ID = 3;
    public static final int MOTOR_RIGHT_SLAVE_ID = 4;
    public static final boolean LEFT_SIDE_INVERTED = false;
    public static final boolean RIGHT_SIDE_INVERTED = true;
      
      // joystick port, may change if there is a mouse or something plugged in, be careful about that
    public static final int JOYSTICK_PORT1 = 0;
	 public static final int JOYSTICK_PORT2 = 1;
    public static final int BALLHANDLER_MOTOR_ID = 6;
    public static final int ROBOT_ARM_MOTOR_ID = 5;

    public static final int ARM_UP_BUTTON = 6;
    public static final int ARM_DOWN_BUTTON = 7;
    public static final int BALL_SHOOTER_BUTTON = 1;
    public static final int BALL_INTAKE_BUTTON = 2;
    public static final int STOP_BUTTON = 3; 
    public static final int RAND_TRAJECTORY = 8; 
    
   }


   public static final class DriveConstants {

    public static final double WHEEL_RADIUS = Units.inchesToMeters(3); //our wheels have a radius of 3 inches
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_RADIUS * 2 * Math.PI;

    public static final double ENCODER_PULSE_DISTANCE (int ENCODER_CPR) {
         return (WHEEL_RADIUS * Math.PI)/ ENCODER_CPR; 
      }

    public static final double TRACK_WIDTH = Units.inchesToMeters(33); // there are .58 meters between the left and right wheels
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);
    public static final double AUTO_VOLTAGE_CONSTRAINT = 7;
    public static final double AUTO_DRIVE_SPEED = 0;

    public static final double ROBOT_ARM_EXPONENT_WEIGHT = 1.5;
    public static final double ROBOT_ARM_DELTA_SENSITIVITY = 20;
    public static final double DEFAULT_DEADBAND = 0.02;

    public static final class Autos {
    public static final double maxVelocity = 4.57; //m/s estimate
    public static final double maxAccel = 1.95; //m/s^2 estimate
    //auto feedforwards (estimated; must do accurate characterization eventually)
    public static final double kS = 0.38; //volts
    public static final double kV = 2.1; //volts*sec/meter
    public static final double kA = 0.25; //volt*sec^2/meter
    }
    //auto feedback constants
    public static final double kP = 0.2;
    public static final double kI = 0.01;
    public static final double kD = 0;
    

   }

   public enum JoystickScaling{
      SQUARED_EXPONENTIAL,
      CUBIC_EXPONENTIAL,
      LINEAR,
      SQUARE_ROOTED,
      CUBE_ROOTED,
      LOGARITHMIC
   }
   
   public enum DriveStyle{
      CUSTOM_TANK,
      NORMAL_ARCADE,
      SATURATED_ARCADE,
      ARCADE_TANK
   }
}