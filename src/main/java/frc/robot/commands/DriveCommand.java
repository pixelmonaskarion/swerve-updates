package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveStyle;
import frc.robot.Constants.JoystickScaling;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;
    private DoubleSupplier turn;
    private DoubleSupplier forward;
  
    
    public DriveCommand(DrivetrainSubsystem drivetrain, DoubleSupplier turn, DoubleSupplier forward) {
        this.drivetrainSubsystem = drivetrain;
        this.turn = turn;
        this.forward = forward;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrainSubsystem.drive(turn.getAsDouble(), forward.getAsDouble(), 0.7, JoystickScaling.LINEAR, DriveStyle.SATURATED_ARCADE);
    }

    @Override
    public void end(boolean interrupted) {
        //same method call as in execute but the turn speed and forward speed are 0
        drivetrainSubsystem.drive(0, 0, 1, JoystickScaling.LINEAR, DriveStyle.NORMAL_ARCADE);

    }
}
