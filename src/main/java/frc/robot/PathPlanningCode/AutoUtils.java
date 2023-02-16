
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
import frc.robot.RobotContainer;
import frc.robot.Commands.VisionTurnCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;


public class AutoUtils {
    private SendableChooser<AutoModes> autoChooser = new SendableChooser<>();
    private final TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics);

    
    public AutoUtils() {
        autoChooser.setDefaultOption("Simple Drive", AutoModes.SIMPLE_DRIVE);
        autoChooser.addOption("Simple Trajectory", AutoModes.SIMPLE_TRAJECTORY);
        autoChooser.addOption("Auto Align Trajectory", AutoModes.AUTO_ALIGN_TRAJECTORY);

        SmartDashboard.putData("auto choices", autoChooser);
    }


    public Command simpleCmdGrp(RobotContainer container) {
        return new RunCommand(() -> container.getDrive().drive(0, 0.8, 0, 0, false, true), container.getDrive()).withTimeout(5);
    }


    public Command simpleTrajectoryCommand(RobotContainer container, Trajectory trajectory) {
      var thetaController = new ProfiledPIDController(
          AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
      SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
          trajectory,
          container.getDrive()::getPose, // Functional interface to feed supplier
          DriveConstants.kDriveKinematics,
  
          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          container.getDrive()::setModuleStates,
          container.getDrive());

      container.getDrive().resetOdometry(trajectory.getInitialPose());

      return swerveControllerCommand.andThen(() -> container.getDrive().mainDrive(0, 0, 0));
    }

    public Command trajectoryAutoAlign(RobotContainer container, Trajectory trajectory) {
      return simpleTrajectoryCommand(container, trajectory)
        .andThen(new VisionTurnCommand(container.getVision(), container.getDrive(), container.getController()));
    }



    public Trajectory simpleCurve() {
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          new Pose2d(3, 0, new Rotation2d(0)),
          config);

      return trajectory;
    }

    public Trajectory driveToScore() {
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(3, 0.5)),
        new Pose2d(2, 2, new Rotation2d(45)),
        config);

      return trajectory;
    }
  


  public Command chooseAuto(RobotContainer container) {
    switch(autoChooser.getSelected()) {
      case SIMPLE_TRAJECTORY:
        return simpleTrajectoryCommand(container, simpleCurve());
      case AUTO_ALIGN_TRAJECTORY:
        return trajectoryAutoAlign(container, driveToScore());
      default:
        return simpleCmdGrp(container);
    }
  }


  public SendableChooser<AutoModes> getChooser() {
    return autoChooser;
  }
    

  private enum AutoModes {
        SIMPLE_DRIVE, SIMPLE_TRAJECTORY, AUTO_ALIGN_TRAJECTORY
  }
}
