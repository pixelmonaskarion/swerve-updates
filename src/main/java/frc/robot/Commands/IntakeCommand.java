package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private IntakeSubsystem intake;
    private double intakeSpeedMultiplier;

    
    //arm down, turn on intake
    public IntakeCommand(IntakeSubsystem intake, double intakeSpeedMultiplier) {
        this.intake = intake;
        this.intakeSpeedMultiplier = intakeSpeedMultiplier;

        addRequirements(intake);
    }


    @Override
    public void execute() {
        System.out.println("IntakeCommand::execute");
        if (Constants.curGamePiece == GamePiece.CONE) {
            intake.pickUpCone(intakeSpeedMultiplier);
        } else if (Constants.curGamePiece == GamePiece.CUBE) {
            intake.pickUpCube(intakeSpeedMultiplier);
        }
    }

    @Override
    public boolean isFinished() {
        System.out.println("IntakeCommand::isFinished");
        return true;
    }

}
