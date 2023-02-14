package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.VisionSubsystem;

public class VisionTurnCommand extends CommandBase {
    private VisionSubsystem vision;
    private DriveSubsystem drive;
    private XboxController controller;
    private PIDController turnController = new PIDController(0.1, 0.2, 0.02);

    public VisionTurnCommand(VisionSubsystem vision, DriveSubsystem drive, XboxController controller) {
        this.vision = vision;
        this.drive = drive;
        this.controller = controller;

        addRequirements(vision, drive);
        //set a limit on overshoot compensation
        turnController.setIntegratorRange(0, Math.toRadians(8));
    }



    @Override
    public void execute() {
        double rotationSpeed = -controller.getRightX();

        var result = vision.getCam().getLatestResult();
            if (result.hasTargets()) {
                rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
            } else {
                rotationSpeed = 0;
            }
      
        drive.drive(MathUtil.applyDeadband(-controller.getLeftY(), 0.06),
        MathUtil.applyDeadband(-controller.getLeftX(), 0.06),
        MathUtil.applyDeadband(rotationSpeed, 0.06),
        MathUtil.applyDeadband(-controller.getRightY(), 0.06),
        controller.getRightBumper(), controller.getAButton());
    }

    @Override
    public boolean isFinished() {
        if (turnController.getPositionError() < Math.toRadians(5)) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0,0,0,0,false, false);
    }
}
