package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeGamePiece;
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
    public void initialize() {
        System.out.println("IntakeCommand::execute");
        intake.intakeGamePiece(intakeSpeedMultiplier);
    }


    @Override
    public void end(boolean interrupted) {
        intake.stopMotors();
    }

}
