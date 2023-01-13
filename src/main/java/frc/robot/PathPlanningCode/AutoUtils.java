package frc.robot.PathPlanningCode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoUtils {

    private GenerateTrajectoryFromFile trajectory = new GenerateTrajectoryFromFile();
    private DrivetrainSubsystem drivetrainSubsystem;

    public AutoUtils(DrivetrainSubsystem drivetrainSubsystem) {
      this.drivetrainSubsystem = drivetrainSubsystem;
    }

    public SequentialCommandGroup runPath(String file_path) {
      
      RamseteCommand ramseteCommand = 
        new RamseteCommand(trajectory.generateTrajectory(
          file_path, 
          GenerateTrajectoryFromFile.createTrajConfig(
            DriveConstants.Autos.maxVelocity, 
            DriveConstants.Autos.maxAccel, 
            0, 
            0, 
            new CentripetalAccelerationConstraint(4))), 
        drivetrainSubsystem::getPose, new RamseteController(), 
        new SimpleMotorFeedforward(DriveConstants.Autos.kS, DriveConstants.Autos.kV, DriveConstants.Autos.kA),
        DriveConstants.DRIVE_KINEMATICS, drivetrainSubsystem::getWheelSpeeds, 
        new PIDController(DriveConstants.kD, DriveConstants.kI, 0),
        new PIDController(DriveConstants.kD, DriveConstants.kI, 0),
        drivetrainSubsystem::tankDriveVolts, drivetrainSubsystem);
      
          //stop robot after autonomous trajectory is followed
      return ramseteCommand.andThen(() -> drivetrainSubsystem.tankDriveVolts(0, 0));
    }

}
