
package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Ultrasonic;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ElevatorConstants;

//TO DO: error handling for if pid controllers get misaligned
public class ElevatorSubsystem extends SubsystemBase {
    private final CANSparkMax elevatorMotor1;
    private final CANSparkMax elevatorMotor2;
    private Ultrasonic distSensor;
   
    private final SparkMaxPIDController m_pidController1;
    private final RelativeEncoder m_encoder1;
    private final RelativeEncoder m_encoder2;
    public static double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, elevatorSpeed;
    //for control via distance sensor
    private final PIDController controller = new PIDController(kP, kI, kD);
    private double curSetpoint;
  
    public ElevatorSubsystem() {
      elevatorMotor1 = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_MASTER, MotorType.kBrushless);
      elevatorMotor2 = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_SLAVE, MotorType.kBrushless);

      distSensor = new Ultrasonic(0, 1);//change to correct channels
      controller.setTolerance(0.3);
      curSetpoint = 0;

      m_encoder1 = elevatorMotor1.getEncoder();
      m_encoder2 = elevatorMotor2.getEncoder();

      elevatorMotor2.follow(elevatorMotor1, true);
      elevatorMotor1.setInverted(true);
      elevatorMotor1.setIdleMode(IdleMode.kBrake);
      elevatorMotor1.restoreFactoryDefaults();

      m_pidController1 = elevatorMotor1.getPIDController();

      // PID coefficients
      kP = 0.1; 
      kI = 1e-4;
      kD = 1; 
      kIz = 0; 
      kFF = 0; 
      kMaxOutput = 1; 
      kMinOutput = -1;
      elevatorSpeed = 0;
  
      // set PID coefficients
      m_pidController1.setP(kP);
      m_pidController1.setI(kI);
      m_pidController1.setD(kD);
      m_pidController1.setIZone(kIz);
      m_pidController1.setFF(kFF);
      m_pidController1.setOutputRange(kMinOutput, kMaxOutput);

  
      // display PID coefficients on SmartDashboard
      SmartDashboard.putNumber("elevator/P Gain", kP);
      SmartDashboard.putNumber("elevator/I Gain", kI);
      SmartDashboard.putNumber("elevator/D Gain", kD);
      SmartDashboard.putNumber("elevator/I Zone", kIz);
      SmartDashboard.putNumber("elevator/Feed Forward", kFF);
      SmartDashboard.putNumber("elevator/Max Output", kMaxOutput);
      SmartDashboard.putNumber("elevator/Min Output", kMinOutput);
      SmartDashboard.putNumber("elevator/Set Rotations", 0);
      SmartDashboard.putNumber("elevator/setpoint", 0);
      SmartDashboard.putNumber("elevator/distance sensor", 0);
      //SmartDashboard.putNumber("elevator motor power", elevatorSpeed);
    }


   
    public void moveElevator(Double setpoint) {
      // read PID coefficients from SmartDashboard
      double p = SmartDashboard.getNumber("elevator/P Gain", 1);
      double i = SmartDashboard.getNumber("elevator/I Gain", 0);
      double d = SmartDashboard.getNumber("elevator/D Gain", 0);
      double iz = SmartDashboard.getNumber("elevator/I Zone", 0);
      double ff = SmartDashboard.getNumber("elevator/Feed Forward", 0);
      double max = SmartDashboard.getNumber("elevator/Max Output", 0);
      double min = SmartDashboard.getNumber("elevator/Min Output", 0);
      double spt = SmartDashboard.getNumber("elevator/setpoint", 0);
  
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
  
      m_pidController1.setReference(spt, CANSparkMax.ControlType.kPosition);

      SmartDashboard.putNumber("elevator/ProcessVariable1", m_encoder1.getPosition());
      SmartDashboard.putNumber("elevator/ProcessVariable2", m_encoder2.getPosition());
    }

    //should be called periodically (in a move command for elevator, in init call setCurSetpoint)
    public void distSensorMove(double setpoint) {
      distSensor.ping();
      double extension = distSensor.getRangeInches() - ElevatorConstants.DISTANCE_SENSOR_OFFSET;
      SmartDashboard.putNumber("elevator/distance sensor", extension);

      if (setpoint != curSetpoint) {
        setCurSetpoint(setpoint);
      }
    
      //calculate control effort required to get elevator from current position to setpoint
      elevatorMotor1.set(MathUtil.clamp(controller.calculate(extension, setpoint), -0.7, 0.7));

      if (controller.atSetpoint()) {
        elevatorMotor1.set(0);
      }
    }

    //setpoint only needs to be updated if a new setpoint is set
    private Runnable isNewSetpoint() {
      return new Runnable() {
        private double pressedLast = getCurSetpoint();
        
        @Override
        public void run() {
          double pressedCur = getCurSetpoint();

          if (pressedLast != pressedCur) {
            setCurSetpoint(getCurSetpoint());
          }
        }
      };
    }

    public void setCurSetpoint(double setpoint) {
      this.curSetpoint = setpoint;
      controller.setSetpoint(setpoint);
    }

    public double getCurSetpoint() {
      return curSetpoint;
    }


    public void simpleMovement(double input) {
      if (input > 0.001) {
        elevatorMotor1.set(ElevatorConstants.downSpeed);
      } else if (input < -0.001) {
        elevatorMotor1.set(ElevatorConstants.upSpeed);
      } else {
        elevatorMotor1.set(0);
      }
    }

    public PIDController getController() {
      return controller;
    }

    public double getCurPosition() {
      return distSensor.getRangeMM()/1000;
    }
 
}