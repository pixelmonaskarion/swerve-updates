package frc.robot.Commands;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.GenericPID.GenericPID;
import frc.robot.GenericPID.PIDConfig;
import frc.robot.Subsystems.ElevatorSubsystem;

//should only be executed if elevator is fully retracted
public class ElevatorPIDCommand extends CommandBase {
    private CANSparkMax motor1;
    private CANSparkMax motor2;

    private PIDConfig pidConfig = new PIDConfig();
    private GenericPID genericPID = new GenericPID(pidConfig);
    private double lastTime = 0.0;
    
    public ElevatorPIDCommand(ElevatorSubsystem elevator) {
        pidConfig.kP = 1.7;
        pidConfig.kI = 0.0001;
        pidConfig.kD = 0.06;
        
        motor1 = elevator.getElevatorMotors().get(Constants.ELEVATOR__MOTOR_ID_MASTER);
        motor2 = elevator.getElevatorMotors().get(Constants.ELEVATOR__MOTOR_ID_SLAVE);
        
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        motor1.getEncoder().setPosition(0);
        motor2.getEncoder().setPosition(0);

        lastTime = Timer.getFPGATimestamp();
        SmartDashboard.putNumber("elevator setpoint", 0);
    }

    @Override
    public void execute() {
        double curTime = Timer.getFPGATimestamp();
        double dt = curTime - lastTime;
        lastTime = curTime;

        double curSetpoint = SmartDashboard.getNumber("elevator setpoint", 0);
        double curPos = (motor1.getEncoder().getPosition() + motor2.getEncoder().getPosition())/(4*Math.PI);

        double controlEffect = genericPID.controlEffect(curSetpoint, curPos, dt);

        motor1.set(controlEffect);
        motor2.set(controlEffect);
    }

    @Override
    public void end(boolean interrupted) {
        motor1.set(0);
        motor2.set(0);
    }
}
