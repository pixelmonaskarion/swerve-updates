package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.ScoringLocation;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;


//should account for piece (cone--y dir offset for placement, cube), scoring location (bottom, mid, high)
public class ScoreGamePieceCommand extends CommandBase {
    private ElevatorSubsystem elevator;
    private IntakeSubsystem intake;
    private XboxController controller;
    private ScoringLocation location;
    private double intakeSpeedMultiplier;

    private Timer timer;
    private double elevatorSetpoint = 0;

    public ScoreGamePieceCommand(ElevatorSubsystem elevator, IntakeSubsystem intake, XboxController controller, ScoringLocation location, double intakeSpeedMultiplier) {
        this.elevator = elevator;
        this.intake = intake;
        this.controller = controller;
        this.location = location;
        this.intakeSpeedMultiplier = intakeSpeedMultiplier;

        timer = new Timer();

        addRequirements(elevator, intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        if (location == ScoringLocation.MID) {
            elevatorSetpoint = 30;
        } else if (location == ScoringLocation.MIDHIGH) {
            elevatorSetpoint = 50;
        } else if (location == ScoringLocation.HIGH) {
            elevatorSetpoint = 102;
        } else if (location == ScoringLocation.SUBSTATION) {
            elevatorSetpoint = 76;
        }
    }

    @Override
    public void execute() {
        elevator.moveElevator(elevatorSetpoint);

        if (timer.get() > 4) {
            if (Constants.curGamePiece == GamePiece.CONE) {
                intake.releaseCone(intakeSpeedMultiplier);
                Constants.curGamePiece = null;
            } else if (Constants.curGamePiece == GamePiece.CUBE) {
                intake.releaseCube(intakeSpeedMultiplier);
                Constants.curGamePiece = null;
            }
        }
    }


    @Override
    public boolean isFinished() {
        if (timer.get() > 6 || controller.getRawAxis(1) != 0 || location == ScoringLocation.LOW) {
            return true;
        }
        return false;
    }


    @Override
    public void end(boolean isFinished) {
        timer.stop();
    }
}