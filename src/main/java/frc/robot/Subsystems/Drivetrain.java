 package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveStyle;
import frc.robot.Constants.JoystickScaling;

public class Drivetrain extends SubsystemBase {
    //initialize motor controllers, encoders, gyro, and differential drive odometry
  private CANSparkMax m_leftMotor_front, m_leftMotor_back, m_rightMotor_front, m_rightMotor_back;
  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;
  private RelativeEncoder leftEncoder, rightEncoder;
  private AnalogGyro gyro;

  private DifferentialDriveOdometry driveOdometry;

  private static double scaleFactorA = 0.4;
  private static double scaleFactorB = 0.55;
  private static double scaleFactorC = 0.35;
  private static double scaleFactorD = 0.6;
  private double leftSpeed;
  private double rightSpeed;

    public Drivetrain() {
        m_leftMotor_front = new CANSparkMax(Constants.MOTOR_LEFT_MASTER_ID, MotorType.kBrushless);
        m_leftMotor_back = new CANSparkMax(Constants.MOTOR_LEFT_SLAVE_ID, MotorType.kBrushless);
        m_rightMotor_front = new CANSparkMax(Constants.MOTOR_RIGHT_MASTER_ID, MotorType.kBrushless);
        m_rightMotor_back = new CANSparkMax(Constants.MOTOR_RIGHT_SLAVE_ID, MotorType.kBrushless);

        leftMotors = new MotorControllerGroup(m_leftMotor_front, m_leftMotor_back);
        rightMotors = new MotorControllerGroup(m_rightMotor_front,m_rightMotor_back);
        rightMotors.setInverted(true);

        leftEncoder = m_leftMotor_front.getEncoder();
        rightEncoder = m_rightMotor_front.getEncoder();

        gyro = new AnalogGyro(Constants.ANALOG_GYRO_PORT);
        
        driveOdometry = new DifferentialDriveOdometry(gyro.getRotation2d(), 0, 0);
        resetEncoders();
    }

    //converts turn and forward speeds to left and right motor speeds
    public void drive(double turn, double forward, JoystickScaling scaling, DriveStyle driveStyle) {
        double turnPower = scaleInputs(turn, scaling);
        double forwardPower = scaleInputs(forward, scaling);

        switch(driveStyle) {
            case CUSTOM_TANK:
                rightSpeed = (turnPower -(-forwardPower-(turnPower/0.5)))/(1+Math.abs(turnPower/0.5));
                leftSpeed =(turnPower +(-forwardPower+(turnPower/0.5)))/(1+Math.abs(turnPower/0.5));

               
                rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);
                leftSpeed = MathUtil.clamp(leftSpeed,-1,1);

                leftMotors.set(leftSpeed);
                rightMotors.set(rightSpeed);

                break;
            case NORMAL_ARCADE:
                leftSpeed = scaleFactorA*Math.abs(turn)+scaleFactorB*turn*turn;
                rightSpeed = scaleFactorC*Math.abs(forward)+scaleFactorD*forward*forward;

                leftMotors.set(leftSpeed);
                rightMotors.set(rightSpeed);
                
                break;
            case ARCADE_TANK:
                leftSpeed = scaleFactorA*Math.abs(turn)+scaleFactorB*turn*turn;
                rightSpeed = scaleFactorC*Math.abs(forward)+scaleFactorD*forward*forward;
                
                if (turn < 0) {
                    leftSpeed *= -1;
                }

                if (forward < 0) {
                    rightSpeed *= -1;
                }

                leftMotors.set(leftSpeed - rightSpeed);
                rightMotors.set(rightSpeed + leftSpeed);
        
                break;
            case SATURATED_ARCADE:
                double saturatedInput;
                double greaterInput = Math.max(Math.abs(turn), Math.abs(forward));
                double smallerInput = Math.min(Math.abs(turn), Math.abs(forward));

                if (greaterInput > 0.0) {
                    saturatedInput = smallerInput/greaterInput + 1;
                } else {
                    saturatedInput = 1.0;
                }

                turnPower = turnPower/saturatedInput;
                forwardPower = forwardPower/saturatedInput;

                rightSpeed = forwardPower+turnPower;
                leftSpeed = turnPower-forwardPower;

                leftMotors.set(leftSpeed);
                rightMotors.set(rightSpeed);

                break;
            default:
                rightSpeed = turnPower-forwardPower;
                leftSpeed = turnPower+forwardPower;

                leftMotors.set(leftSpeed);
                rightMotors.set(rightSpeed);
        }
    }

    //scale joystick movements
    public double scaleInputs(double rawInput, JoystickScaling scaleType) {
        double scaledValue = rawInput;
        scaledValue = MathUtil.applyDeadband(scaledValue, Constants.DEFAULT_DEADBAND);

        switch(scaleType) {
            case QUADRATIC:
                scaledValue *= Math.abs(scaledValue);
                break;
            case CUBIC:
                scaledValue *= scaledValue*scaledValue;
                break;
            case LOGARITHMIC:
                scaledValue = (scaledValue != 0) ? (Math.log(Math.abs(scaledValue))/Math.log(Math.E)) : 0;
                break;
            case SQRT:
                scaledValue = Math.sqrt(Math.abs(scaledValue));
                break;
            case CBRT:
                scaledValue = Math.cbrt(Math.abs(scaledValue));
                break;
            default:
            return scaledValue;
        }
        return MathUtil.clamp(scaledValue, -1, 1);
    }

    //simple drive method for auto testing, etc
    public void testDrive(double left, double right) {
        leftMotors.set(left);
        rightMotors.set(right);
    }

    //tank drive volts method solely for following trajectories
    public void tankDriveVolts(double leftVolts, double rightVolts){
        double batteryVoltage = RobotController.getBatteryVoltage();
        if (Math.max(Math.abs(leftVolts),Math.abs(rightVolts))> batteryVoltage ){
          leftVolts *= batteryVoltage /12.0;
          rightVolts *= batteryVoltage/ 12.0;
        }
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        double cmPerTick =  Constants.WHEEL_CIRCUMFERENCE * 100 / (leftEncoder.getCountsPerRevolution() *4);
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity()*cmPerTick/100,rightEncoder.getVelocity()*cmPerTick/100);
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public Pose2d getPose() {
        return driveOdometry.getPoseMeters();
    }

    //call periodically during autonomous
    public void updateOdometry() {
        driveOdometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
    }
}

