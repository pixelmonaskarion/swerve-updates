package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Commands.SimpleDriveCommand;
import frc.robot.PathPlanningCode.AutoUtils;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {
    private Joystick joystick = new Joystick(Constants.JOYSTICK_PORT1);

    private final Drivetrain drivetrain = new Drivetrain();

    private final PowerDistribution powerDistribution = new PowerDistribution();
    private final AutoUtils autoUtils = new AutoUtils();
    
    public RobotContainer() {
        powerDistribution.clearStickyFaults();

        drivetrain.setDefaultCommand(new SimpleDriveCommand(drivetrain, 
        () -> -joystick.getRawAxis(1),
        () -> -joystick.getRawAxis(0)));
    }

    public Drivetrain getRobotDrive() {
        return drivetrain;
    }

    public AutoUtils getAutoPath() {
        return autoUtils;
    }

    public Command getAutonomousCommand() {
        return new RunCommand(() -> drivetrain.testDrive(0.1, 0.1), drivetrain).withTimeout(5);
    }
}
