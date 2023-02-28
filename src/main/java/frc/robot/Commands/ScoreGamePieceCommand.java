package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ScoringLocation;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;


//should account for piece (cone--y dir offset for placement, cube), scoring location (bottom, mid, high)
public class ScoreGamePieceCommand extends CommandBase {
    private ArmSubsystem arm;
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private ScoringLocation location;
    private double intakeSpeedMultiplier;

    private Timer timer;
    private double elevatorSetpoint = 0;

    public ScoreGamePieceCommand(ArmSubsystem arm, ElevatorSubsystem elevator, IntakeSubsystem intake, ScoringLocation location, double intakeSpeedMultiplier) {
        this.arm = arm;
        this.elevator = elevator;
        this.intake = intake;
        this.location = location;
        this.intakeSpeedMultiplier = intakeSpeedMultiplier;

        timer = new Timer();

        addRequirements(arm, elevator, intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        if (location == ScoringLocation.MID) {
            elevatorSetpoint = 0.5;
        } else if (location == ScoringLocation.MIDHIGH) {
            elevatorSetpoint = 1;
        } else if (location == ScoringLocation.HIGH) {
            elevatorSetpoint = 1.5;
        }
    }

    @Override
    public void execute() {
        arm.expand();
        elevator.moveElevator(() -> elevatorSetpoint);

        if (timer.get() > 4) {
            intake.releaseCargo(intakeSpeedMultiplier);
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
