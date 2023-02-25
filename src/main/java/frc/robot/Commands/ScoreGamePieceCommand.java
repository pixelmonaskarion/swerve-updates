package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ScoringLocation;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;


//should account for piece (cone--y dir offset for placement, cube), scoring location (bottom, mid, high)
public class ScoreGamePieceCommand extends CommandBase {
    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;
    private ScoringLocation location;

    private Timer timer;
    private double elevatorSetpoint = 0;

    public ScoreGamePieceCommand(ArmSubsystem arm, ElevatorSubsystem elevator, ScoringLocation location) {
        this.arm = arm;
        this.elevator = elevator;
        this.location = location;

        timer = new Timer();

        addRequirements(arm, elevator);
    }

    @Override
    public void initialize() {
        timer.start();

        if (location == ScoringLocation.MID) {
            elevatorSetpoint = 1;
        } else if (location == ScoringLocation.HIGH) {
            elevatorSetpoint = 2;
        }
    }

    @Override
    public void execute() {
        arm.expand();
        elevator.moveElevator(() -> elevatorSetpoint);

        if (timer.get() > 4) {
            //ballhander motor off
        }
    }


    @Override
    public boolean isFinished() {
        if (timer.get() > 6) {
            return true;
        }
        return false;
    }


    @Override
    public void end(boolean isFinished) {
        timer.stop();
    }
}
