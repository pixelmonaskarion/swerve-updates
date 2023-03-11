package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Subsystems.ElevatorSubsystem;

public class ElevatorCommand extends CommandBase {
    private ElevatorSubsystem elevator;
    private double setpoint;

    public ElevatorCommand(ElevatorSubsystem elevator, double setpoint) {
        this.elevator = elevator;
        this.setpoint = setpoint;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setCurSetpoint(setpoint);
    }

    @Override
    public void execute() {
        elevator.distSensorMove(setpoint);
    }

    
    @Override
    public boolean isFinished() {
        if (elevator.getController().atSetpoint() || elevator.getCurPosition() == ElevatorConstants.MAX_EXTENSION) {
            return true;
        }
        return false;
    }
}
