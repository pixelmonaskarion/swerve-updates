package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.Arm;

public class ArmCommand extends CommandBase {
    private Arm arm;
    private double setpoint;

    public ArmCommand(Arm arm, double setpoint) {
        this.arm = arm;
        this.setpoint = setpoint;

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.enable();
        arm.setGoal(setpoint);
    }

    @Override
    public boolean isFinished() {
        if (Math.abs(arm.getArmMotor().get()) < 0.1) {
            return true;
        }
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        arm.disable();
    }
}
