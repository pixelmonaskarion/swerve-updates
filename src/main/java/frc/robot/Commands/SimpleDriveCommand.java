package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveStyle;
import frc.robot.Constants.JoystickScaling;
import frc.robot.Subsystems.Drivetrain;

public class SimpleDriveCommand extends CommandBase {
    private Drivetrain drivetrain;
    private DoubleSupplier turn;
    private DoubleSupplier forward;

    public SimpleDriveCommand(Drivetrain drivetrain, DoubleSupplier turn, DoubleSupplier forward) {
        this.drivetrain = drivetrain;
        this.turn = turn;
        this.forward = forward;

        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.drive(turn.getAsDouble(), forward.getAsDouble(), JoystickScaling.LINEAR, DriveStyle.ARCADE_TANK);
    }


    @Override
    public void end(boolean interrupted) {
        drivetrain.testDrive(0, 0);
    }
}
