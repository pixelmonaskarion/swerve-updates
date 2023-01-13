package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveStyle;
import frc.robot.Constants.JoystickScaling;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SpinCommand extends CommandBase {
    private DrivetrainSubsystem drivetrainSubsystem;
    private double spinTime;
    private Timer timer;

    public SpinCommand(DrivetrainSubsystem drivetrainSubsystem, double spinTime) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.spinTime = spinTime;
        timer = new Timer();
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        drivetrainSubsystem.drive(0.5, 0, 0.8, JoystickScaling.LINEAR, DriveStyle.NORMAL_ARCADE);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(0, 0, 0.0, JoystickScaling.LINEAR, DriveStyle.NORMAL_ARCADE);
    }

    @Override
    public boolean isFinished() {
        if (timer.get() > spinTime) {
            return true;
        }
        return false;
    }
}

