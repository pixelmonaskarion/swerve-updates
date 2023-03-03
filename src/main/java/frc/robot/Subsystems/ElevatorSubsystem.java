package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

//to do: error handling for if pid controllers get misaligned
public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax elevatorMotor1;
    private final CANSparkMax elevatorMotor2;
   
    private final SparkMaxPIDController m_pidController1;
    private final SparkMaxPIDController m_pidController2;
    private final RelativeEncoder m_encoder1;
    private final RelativeEncoder m_encoder2;
    public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    private List<CANSparkMax> motorList = new ArrayList<>();
  
    public ElevatorSubsystem() {
      elevatorMotor1 = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID_MASTER, MotorType.kBrushless);
      elevatorMotor2 = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID_SLAVE, MotorType.kBrushless);
      m_encoder1 = elevatorMotor1.getEncoder();
      m_encoder2 = elevatorMotor2.getEncoder();

      elevatorMotor1.setIdleMode(IdleMode.kBrake);
      elevatorMotor1.restoreFactoryDefaults();
      elevatorMotor1.setSoftLimit(SoftLimitDirection.kForward, 51);
      elevatorMotor1.setSoftLimit(SoftLimitDirection.kReverse, 51);
      motorList.add(elevatorMotor1);
      elevatorMotor2.setIdleMode(IdleMode.kBrake);
      elevatorMotor2.restoreFactoryDefaults();
      elevatorMotor2.setSoftLimit(SoftLimitDirection.kForward, 51);
      elevatorMotor2.setSoftLimit(SoftLimitDirection.kReverse, 51);
      motorList.add(elevatorMotor2);

      m_pidController1 = elevatorMotor1.getPIDController();
      m_pidController2 = elevatorMotor2.getPIDController();

      // PID coefficients
      kP = 0.1; 
      kI = 1e-4;
      kD = 1; 
      kIz = 0; 
      kFF = 0; 
      kMaxOutput = 1; 
      kMinOutput = -1;
  
      // set PID coefficients
      m_pidController1.setP(kP);
      m_pidController1.setI(kI);
      m_pidController1.setD(kD);
      m_pidController1.setIZone(kIz);
      m_pidController1.setFF(kFF);
      m_pidController1.setOutputRange(kMinOutput, kMaxOutput);
      m_pidController2.setP(kP);
      m_pidController2.setI(kI);
      m_pidController2.setD(kD);
      m_pidController2.setIZone(kIz);
      m_pidController2.setFF(kFF);
      m_pidController2.setOutputRange(kMinOutput, kMaxOutput);
  
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("P Gain", kP);
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIz);
      SmartDashboard.putNumber("Feed Forward", kFF);
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);
      SmartDashboard.putNumber("Set Rotations", 0);
    }
  
   
    public void moveElevator(DoubleSupplier joystickPos) {
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double ff = SmartDashboard.getNumber("Feed Forward", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);
      double rotations = MathUtil.clamp(joystickPos.getAsDouble(), -0.1, 0.1);
  
      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { m_pidController1.setP(p); kP = p; }
      if((i != kI)) { m_pidController1.setI(i); kI = i; }
      if((d != kD)) { m_pidController1.setD(d); kD = d; }
      if((iz != kIz)) { m_pidController1.setIZone(iz); kIz = iz; }
      if((ff != kFF)) { m_pidController1.setFF(ff); kFF = ff; }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
        m_pidController1.setOutputRange(min, max); 
        kMinOutput = min; kMaxOutput = max; 
      }
  
      m_pidController1.setReference(rotations, CANSparkMax.ControlType.kPosition);
      m_pidController2.setReference(rotations, CANSparkMax.ControlType.kPosition);
      
      SmartDashboard.putNumber("SetPoint", rotations);
      SmartDashboard.putNumber("ProcessVariable1", m_encoder1.getPosition());
      SmartDashboard.putNumber("ProcessVariable2", m_encoder2.getPosition());
    }

  
    public void movementTest(boolean extend, boolean retract) {
      if (extend) {
        elevatorMotor1.set(0.1);
        elevatorMotor2.set(-0.1);
      }

        if (retract) {
        elevatorMotor1.set(-0.1);
        elevatorMotor2.set(0.1);
      }
    }

    
  
    public List<CANSparkMax> getElevatorMotors() {
      return motorList;
    }
  
}