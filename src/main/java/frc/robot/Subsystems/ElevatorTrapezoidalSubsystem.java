package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ElevatorConstants;



public class ElevatorTrapezoidalSubsystem extends ProfiledPIDSubsystem {
    private final CANSparkMax elevatorMotor1;
    private final CANSparkMax elevatorMotor2;
    private final RelativeEncoder encoder;

    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
        ElevatorConstants.ELEVATOR_kS, ElevatorConstants.ELEVATOR_kG, ElevatorConstants.ELEVATOR_kV, ElevatorConstants.ELEVATOR_kA);

    public ElevatorTrapezoidalSubsystem() {
       super(new ProfiledPIDController(1,0,0, new TrapezoidProfile.Constraints(2, 3)));
       
       elevatorMotor1 = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_MASTER, MotorType.kBrushless);
       elevatorMotor2 = new CANSparkMax(ElevatorConstants.ELEVATOR_MOTOR_ID_SLAVE, MotorType.kBrushless);

       elevatorMotor2.follow(elevatorMotor1, true);
       elevatorMotor1.setInverted(true);
       elevatorMotor1.setIdleMode(IdleMode.kBrake);
       elevatorMotor1.restoreFactoryDefaults();

       encoder = elevatorMotor1.getEncoder();
       resetEncoders();
    }

    @Override
    protected void useOutput(double output, State setpoint) {
        double feed = feedforward.calculate(setpoint.position, setpoint.velocity);
        elevatorMotor1.set(output + feed);
    }

    @Override
    protected double getMeasurement() {
        return encoder.getPosition();
    }

    public void resetEncoders() {
        encoder.setPosition(0);
    }

    
}
