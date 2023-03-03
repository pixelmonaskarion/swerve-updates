package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private ArmSubsystem arm;
    private IntakeSubsystem intake;
    private double intakeSpeedMultiplier;

    
    //arm down, turn on intake
    public IntakeCommand(ArmSubsystem arm, IntakeSubsystem intake, double intakeSpeedMultiplier) {
        this.arm = arm;
        this.intake = intake;
        this.intakeSpeedMultiplier = intakeSpeedMultiplier;

        addRequirements(arm, intake);
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new InstantCommand(arm::retract));
    }

    @Override
    public void execute() {
        if (Constants.curGamePiece == GamePiece.CONE) {
            intake.pickUpCone(intakeSpeedMultiplier);
        } else if (Constants.curGamePiece == GamePiece.CUBE) {
            intake.pickUpCube(intakeSpeedMultiplier);
        }
    }
}
