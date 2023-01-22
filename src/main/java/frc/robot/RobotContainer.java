package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Commands.SimpleDriveCommand;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {
    private Joystick joystick = new Joystick(Constants.JOYSTICK_PORT1);

    private final Drivetrain drivetrain = new Drivetrain();
    private final PowerDistribution powerDistribution = new PowerDistribution();
    
    public RobotContainer() {
        powerDistribution.clearStickyFaults();

        drivetrain.setDefaultCommand(new SimpleDriveCommand(drivetrain, 
        () -> -joystick.getY(),
        () -> -joystick.getX()));
    }

    public Drivetrain getRobotDrive() {
        return drivetrain;
    }

    public Command getAutonomousCommand() {
        return new RunCommand(() -> drivetrain.testDrive(0.5, 0.5), drivetrain).withTimeout(5);
    }
}
