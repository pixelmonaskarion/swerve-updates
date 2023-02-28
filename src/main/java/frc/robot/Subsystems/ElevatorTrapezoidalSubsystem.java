package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

import frc.robot.Constants;


public class ElevatorTrapezoidalSubsystem extends TrapezoidProfileSubsystem {

    private final CANSparkMax elevatorMotor1;
    private final CANSparkMax elevatorMotor2;
    private final SparkMaxPIDController m_pidController1;
    private final SparkMaxPIDController m_pidController2;
    private final RelativeEncoder m_encoder;
    private boolean resetPosition;

    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(Constants.ELEVATOR_kS, Constants.ELEVATOR_kG, Constants.ELEVATOR_kV);

    public ElevatorTrapezoidalSubsystem(Constraints constraints) {
        super(constraints);
        
        elevatorMotor1 = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID_MASTER, MotorType.kBrushless);
        elevatorMotor2 = new CANSparkMax(Constants.ELEVATOR_MOTOR_ID_SLAVE, MotorType.kBrushless);
        resetPosition = false;

        elevatorMotor1.restoreFactoryDefaults();
        elevatorMotor2.restoreFactoryDefaults();
        m_encoder = elevatorMotor1.getEncoder();
        m_encoder.setPosition(0);
        m_pidController1 = elevatorMotor1.getPIDController();
        m_pidController2 = elevatorMotor2.getPIDController();

        m_pidController1.setP(1);
        m_pidController1.setI(0);
        m_pidController1.setD(0);
        m_pidController2.setP(1);
        m_pidController2.setI(0);
        m_pidController2.setD(0);
    }

    @Override
    protected void useState(State state) {
        double ff = feedforward.calculate(state.position, state.velocity);

        if (resetPosition) {
            state.position = m_encoder.getPosition();
            resetPosition = false;
        }

        m_pidController1.setReference(state.position + ff, ControlType.kPosition);
        m_pidController2.setReference(state.position + ff, ControlType.kPosition);
    }

    @Override 
    public void periodic() {
        super.periodic();
    }

    public double getElevatorLength() {
        return m_encoder.getPosition();
    }
    
    public void setElevatorLength(double goal) {
        setGoal(goal);
    }

    public void resetElevatorExtension() {
        setGoal(getElevatorLength());
        resetPosition = true;
    }
}
