package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveStyle;
import frc.robot.Constants.JoystickScaling;
import frc.robot.Constants.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DrivetrainSubsystem extends SubsystemBase {
    private CANSparkMax m_leftDriveFront, m_leftDriveBack;
    private CANSparkMax m_rightDriveFront, m_rightDriveBack;
    private MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;
    private boolean breakStatus;
    private final DifferentialDriveOdometry m_odometry;
    private double cmPerTick;
    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;
    private DrivetrainOptions drivetrain;
    
    public DrivetrainSubsystem() {
    System.out.println("method called");
  
        m_leftDriveFront = new CANSparkMax(RobotMap.MOTOR_LEFT_MASTER_ID, MotorType.kBrushless);
        m_leftDriveBack= new CANSparkMax(RobotMap.MOTOR_LEFT_SLAVE_ID, MotorType.kBrushless);
        m_rightDriveFront= new CANSparkMax(RobotMap.MOTOR_RIGHT_MASTER_ID, MotorType.kBrushless);
        m_rightDriveBack = new CANSparkMax(RobotMap.MOTOR_RIGHT_SLAVE_ID, MotorType.kBrushless); 
        leftEncoder= m_leftDriveFront.getEncoder();
        rightEncoder= m_rightDriveFront.getEncoder();
        leftMotors = new MotorControllerGroup(m_leftDriveFront, m_leftDriveBack);
        rightMotors = new MotorControllerGroup(m_rightDriveFront, m_rightDriveBack);
        drivetrain = new DrivetrainOptions(leftMotors, rightMotors);
        
        breakStatus = false;
      
        resetEncoders();
        m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

        
    }

    public void setBreakStatus(boolean breakOn){
        if (breakOn != breakStatus && breakOn == true){
          m_leftDriveFront.setIdleMode(IdleMode.kBrake);
          m_leftDriveBack.setIdleMode(IdleMode.kBrake);
          m_rightDriveFront.setIdleMode(IdleMode.kBrake);
          m_rightDriveBack.setIdleMode(IdleMode.kBrake);
        }
        else if(breakOn != breakStatus && breakOn == false){
          m_leftDriveFront.setIdleMode(IdleMode.kCoast);
          m_leftDriveBack.setIdleMode(IdleMode.kCoast);
          m_rightDriveFront.setIdleMode(IdleMode.kCoast);
          m_rightDriveBack.setIdleMode(IdleMode.kCoast);
        }
      }

    //calls drive method from drivetrainOptions
    public void drive(double turn, double forward, double scaling, JoystickScaling joystickStyle, DriveStyle driveType) {
        drivetrain.pickDrivetrain(turn, forward, joystickStyle, scaling, driveType);
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public double getAverageEncoderDistance() {
        return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
    }
    
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    //until we get a gyroscope, this should work to get rotation angle
    public double getHeading() {
        cmPerTick =  DriveConstants.WHEEL_CIRCUMFERENCE * 100 / (leftEncoder.getCountsPerRevolution() *4);
        double degreesPerTick = cmPerTick / (DriveConstants.WHEEL_RADIUS * 100) * (180 * Math.PI);
        
        double encoderDifference = leftEncoder.getPosition() - rightEncoder.getPosition();
        double turningValue = encoderDifference * degreesPerTick;
        
        double finalDegrees = (turningValue % 360) * -1;
        return finalDegrees;
        
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity()*cmPerTick/100,rightEncoder.getVelocity()*cmPerTick/100);
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

        drivetrain.feed();
    }
    
}
