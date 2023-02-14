
package frc.robot.PathPlanningCode;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.DriveSubsystem;

public class AutoUtils {
    private SendableChooser<AutoModes> autoChooser = new SendableChooser<>();
    
    public AutoUtils() {
        autoChooser.setDefaultOption("Simple Drive", AutoModes.SIMPLE_DRIVE);


        SmartDashboard.putData("auto choices", autoChooser);
    }

    public Command simpleCmdGrp(DriveSubsystem m_robotDrive) {
        return new RunCommand(() -> m_robotDrive.drive(0, 0.8, 0, 0, false, true), m_robotDrive).withTimeout(5);
    }


    public Command simpleTrajectoryCommand(DriveSubsystem m_robotDrive) {
      TrajectoryConfig config = new TrajectoryConfig(
          AutoConstants.kMaxSpeedMetersPerSecond,
          AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          .setKinematics(DriveConstants.kDriveKinematics);
  
      Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          new Pose2d(3, 0, new Rotation2d(0)),
          config);
  
      var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          exampleTrajectory,
          m_robotDrive::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,
  
          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          m_robotDrive::setModuleStates,
          m_robotDrive);

      m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

      return swerveControllerCommand.andThen(() -> m_robotDrive.mainDrive(0, 0, 0));
    }

  


  public Command chooseAuto(DriveSubsystem m_robotDrive) {
    switch(autoChooser.getSelected()) {
      case SIMPLE_TRAJECTORY:
        return simpleTrajectoryCommand(m_robotDrive);
      default:
        return simpleCmdGrp(m_robotDrive);
    }
  }

  public SendableChooser<AutoModes> getChooser() {
    return autoChooser;
  }
    

    private enum AutoModes {
        SIMPLE_DRIVE, SIMPLE_TRAJECTORY
    }
}
