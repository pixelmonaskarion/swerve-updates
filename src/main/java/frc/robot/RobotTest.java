package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Subsystems.DriveSubsystem;

public class RobotTest extends TimedRobot {
    DriveSubsystem drive = new DriveSubsystem();
    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("lol");

    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 0.1, 0.2);


    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    XboxController controller = new XboxController(0);

    @Override
    public void teleopPeriodic() {
        double forwardSpeed;
        double rotationSpeed;

        if (controller.getBButton()) {
            
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();
            
            if (result.hasTargets()) {
                double range = PhotonUtils.calculateDistanceToTargetMeters(0.15, 0.435, 0, 
                Units.degreesToRadians(result.getBestTarget().getPitch()));

                forwardSpeed = forwardController.calculate(range, 0);
                // Calculate angular turn power
                // -1.0 required to ensure positive PID controller effort _increases_ yaw
                rotationSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0);
                
            } else {
                // If we have no targets, stay still.
                forwardSpeed = 0;
                rotationSpeed = 0;
            }
        } else {
            // Manual Driver Mode
            forwardSpeed = controller.getLeftY();
            rotationSpeed = controller.getRightX();
        }

        System.out.printf("%f forward speed, %f rotation speed %n", forwardSpeed, rotationSpeed);

        // Use our forward/turn speeds to control the drivetrain
        drive.mainDrive(MathUtil.applyDeadband(-controller.getLeftY(), 0.06),
        MathUtil.applyDeadband(-controller.getLeftX(), 0.06),
        MathUtil.applyDeadband(rotationSpeed, 0.06));
    }
}





