package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeGamePieceCommand extends CommandBase {
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private double intakeSpeedMultiplier;

    private final Timer timer;
    
    //arm down, turn on intake
    public IntakeGamePieceCommand(ArmSubsystem arm, IntakeSubsystem intake, double intakeSpeedMultiplier) {
        this.arm = arm;
        this.intake = intake;
        this.intakeSpeedMultiplier = intakeSpeedMultiplier;

        timer = new Timer();
        addRequirements(arm, intake);
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new InstantCommand(arm::retract));

        timer.start();
    }

    @Override
    public void execute() {
        intake.pickUpCargo(intakeSpeedMultiplier);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3) ? true : false;
    }
}
