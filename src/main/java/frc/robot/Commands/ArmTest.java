package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmSubsystem;

public class ArmTest extends CommandBase {
    private ArmSubsystem arm;
    
    public ArmTest(ArmSubsystem arm) {
        this.arm = arm;
    }

    @Override
    public void execute() {
        arm.expand();
    }
}
