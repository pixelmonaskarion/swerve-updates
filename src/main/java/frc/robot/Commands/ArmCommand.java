package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
    private ArmSubsystem arm;
    private double setpoint;

    public ArmCommand(ArmSubsystem arm, double setpoint) {
        this.arm = arm;
        this.setpoint = setpoint;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.moveArm(() -> setpoint);
    }

    
    @Override
    public void end(boolean interrupted) {
        arm.getArmMotor().set(0.05);
    }
}
