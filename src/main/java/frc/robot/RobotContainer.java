package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ArmCommand;
import frc.robot.Commands.SimpleDriveCommand;
import frc.robot.PathPlanningCode.AutoUtils;
import frc.robot.Subsystems.Arm;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.SimpleArm;

public class RobotContainer {
    private Joystick joystick = new Joystick(Constants.JOYSTICK_PORT1);
    private Joystick armJoystick = new Joystick(Constants.JOYSTICK_PORT2);

    private final Drivetrain drivetrain = new Drivetrain();
    //private final SimpleArm simpleArm = new SimpleArm();
    private final Arm arm = new Arm(1.31);

    private final PowerDistribution powerDistribution = new PowerDistribution();
    private final AutoUtils autoUtils = new AutoUtils();
    
    public RobotContainer() {
        powerDistribution.clearStickyFaults();

        startDefaultCommands();
        configureButtonBindings();
    }

    public void startDefaultCommands() {
        drivetrain.setDefaultCommand(new SimpleDriveCommand(drivetrain, 
        () -> -joystick.getRawAxis(1),
        () -> -joystick.getRawAxis(0)));

        //simpleArm.setDefaultCommand(new RunCommand(() -> simpleArm.moveArm(getArmToggle())));
    }

    public int getArmToggle() {
        if (armJoystick.getRawButton(6)) {
            return 6;
        } else if (armJoystick.getRawButton(7)) {
            return 7;
        } else {
            return -1;
        }
    }

    public void configureButtonBindings() {
        
        new Trigger(() -> armJoystick.getRawButton(6))
            .onTrue(new ArmCommand(arm));

        //simple trigger to test if it works for joysticks
        new Trigger(() -> armJoystick.getRawButton(5))
            .whileTrue((new RunCommand(() -> drivetrain.testDrive(0.2, 0.2), drivetrain)));
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
