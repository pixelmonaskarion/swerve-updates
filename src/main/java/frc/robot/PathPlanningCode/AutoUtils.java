
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
import frc.robot.Commands.VisionTranslateCommand;
import frc.robot.Commands.VisionTurnCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

 //trajectory -- command:
    /*
     * priority 1 auto:
     * go forward a bit -- score gamepiece command (arm up, elevator extended)
     * drive backwards (out of community, dependent on starting position)
     * 
     * priority 2 auto:
     * go forward a bit -- score gamepiece
     * rotate 180 degrees -- charge station balance command (tiny backup, arm down, elevator extended forward)
     * get onto charge station (dependent on starting position)
     * 
     * priority 3 auto:
     * go forward a bit -- score gamepiece
     * rotation 180 degrees
     * drive to staged gamepiece -- intake staged gamepiece command (cone or cube)
     * 
     * priorty 4 auto:
     * priority 3 auto + balance on charge station
     * 
     * 
     * 
     * starting positions: left CS (charge station), center CS, right CS
     * 
     * assuming +x dir is forward and +y dir is right
     */


public class AutoUtils {
    private SendableChooser<AutoModes> autoChooser = new SendableChooser<>();
    private SendableChooser<StartPos> startPosChooser = new SendableChooser<>();

    private final TrajectoryConfig config = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(DriveConstants.kDriveKinematics);

    
    public AutoUtils() {
      autoChooser.setDefaultOption("Simple Drive", AutoModes.SIMPLE_DRIVE);
      autoChooser.addOption("Simple Trajectory", AutoModes.SIMPLE_TRAJECTORY);
      autoChooser.addOption("Auto Align Trajectory", AutoModes.AUTO_ALIGN_TRAJECTORY);
      autoChooser.addOption("Priority 1 Auto", AutoModes.PRIORITY_1_AUTO);
      autoChooser.addOption("Priority 2 Auto", AutoModes.PRIORITY_2_AUTO);
      autoChooser.addOption("Priority 3 Auto", AutoModes.PRIORITY_3_AUTO);
      autoChooser.addOption("Priority 4 Auto", AutoModes.PRIORITY_4_AUTO);

      startPosChooser.setDefaultOption("Left of charge station", StartPos.LEFT_CS);
      startPosChooser.addOption("Middle of charge station", StartPos.MID_CS);
      startPosChooser.addOption("Right of charge station", StartPos.RIGHT_CS);

      SmartDashboard.putData("auto choices", autoChooser);
      SmartDashboard.putData("start position choices", startPosChooser);
    }


    public Command simpleCmdGrp(RobotContainer container) {
        return new RunCommand(() -> container.getDrive().mainDrive(0.8, 0, 0), container.getDrive()).withTimeout(2);
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

    //to do: write score command (if for low level, no additional code is needed)
    public Command priorityOneAuto(RobotContainer container, StartPos startPos) {
      return simpleTrajectoryCommand(container, initDriveToScore())
        //.andThen(new ScoreGamePieceCommand())
      .andThen(simpleTrajectoryCommand(container, driveOutOfCommunity(startPos)));
    }

    //to do: write charge station balance command, is rotation180 breaking it?
    public Command priorityTwoAuto(RobotContainer container, StartPos startPos) {
      return simpleTrajectoryCommand(container, initDriveToScore())
        .andThen(simpleTrajectoryCommand(container, rotate180().concatenate(getOnChargeStation(startPos))));
            //.alongWith(new ChargeStationBalanceCommand()));
    }

    public Command priorityThreeAuto(RobotContainer container, StartPos startPos) {
      return simpleTrajectoryCommand(container, initDriveToScore())
        .andThen(simpleTrajectoryCommand(container, rotate180().concatenate(driveToStagedGamePiece(startPos))))
        //andThen(new IntakeGamePieceCommand())
        .andThen(simpleTrajectoryCommand(container, rotate180()))
        .andThen(new VisionTranslateCommand(container.getVision(), container.getDrive(), container.getController()));
        //andThen(ScoreGamePieceCommand())
    }

    public Command priorityFourAuto(RobotContainer container, StartPos startPos) {
      return priorityThreeAuto(container, startPos)
        .andThen(simpleTrajectoryCommand(container, rotate180().concatenate(getOnChargeStation(startPos))));
    }

    //test
    private Trajectory simpleCurve() {
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
          new Pose2d(3, 0, new Rotation2d(0)),
          config);

      return trajectory;
    }

    //test
    private Trajectory driveToScore() {
      Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(3, 0.5)),
        new Pose2d(2, 2, new Rotation2d(45)),
        config);

      return trajectory;
    }

    private Trajectory rotate180() {
      try{
      return TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0,0, new Rotation2d(180))), config);
      } catch (ArrayIndexOutOfBoundsException e) {
        e.printStackTrace();
      }
      return null;
    }
    
    //drive 1 m forward
    private Trajectory initDriveToScore() {
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(new Translation2d(0.5, 0)),
          new Pose2d(0.5, 0, new Rotation2d(0)),
          config);
  
      return trajectory;
    }

    //back up 5 meters or drive around charge station
    private Trajectory driveOutOfCommunity(StartPos pos) {
      Trajectory trajectory;
      if (pos == StartPos.LEFT_CS || pos == StartPos.RIGHT_CS) {
        trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0,0,new Rotation2d(0)), 
          List.of(new Translation2d(-5, 0.1)),
          new Pose2d(-5.1, 0.2, new Rotation2d(0)), config);
        return trajectory;
      } else if (pos == StartPos.MID_CS) {
        trajectory = TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, new Rotation2d(0)), 
          List.of(new Translation2d(-3.9, -4)), 
          new Pose2d(-4, -4, new Rotation2d(0)), config);
        return trajectory;
      }
      return null;
    }

    private Trajectory getOnChargeStation(StartPos pos) {
      Trajectory trajectory;
        if (pos == StartPos.LEFT_CS) {
          trajectory = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(1, 2, new Rotation2d(0)), new Pose2d(0, -0.3, new Rotation2d(0))),
            config);
          return trajectory;
        } else if (pos == StartPos.MID_CS) {
          trajectory = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(2.9, 0, new Rotation2d(0))), config);
          return trajectory;
        } else if (pos == StartPos.RIGHT_CS) {
          trajectory = TrajectoryGenerator.generateTrajectory(
            List.of(new Pose2d(1, -2, new Rotation2d(0)), new Pose2d(0, 0.3, new Rotation2d(0))), config);
          return trajectory;
        }
        return null;
    }

    private Trajectory driveToStagedGamePiece(StartPos pos) {
      Trajectory trajectory;
        if (pos == StartPos.LEFT_CS) {
          trajectory = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(5.9, 0.2, new Rotation2d(0))), config);
          return trajectory;
        } else if (pos == StartPos.MID_CS) {
          trajectory = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0, 1.8, new Rotation2d(0)), new Pose2d(5.9, 0, new Rotation2d(0))), config);
          return trajectory;
        } else if (pos == StartPos.RIGHT_CS) {
          trajectory = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(5.9, -0.4, new Rotation2d(0))), config);
          return trajectory;
        }
        return null;
    }


  //start position chooser is fed into this for start position-dependent trajectories
  public Command chooseAuto(RobotContainer container, StartPos startPos) {
    switch(autoChooser.getSelected()) {
      case SIMPLE_TRAJECTORY:
        return simpleTrajectoryCommand(container, simpleCurve());
      case AUTO_ALIGN_TRAJECTORY:
        return trajectoryAutoAlign(container, driveToScore());
      case PRIORITY_1_AUTO:
        return priorityOneAuto(container, startPos);
      case PRIORITY_2_AUTO:
        return priorityTwoAuto(container, startPos);
      case PRIORITY_3_AUTO:
        return priorityThreeAuto(container, startPos);
      case PRIORITY_4_AUTO:
        return priorityFourAuto(container, startPos);
      default:
        return simpleCmdGrp(container);
    }
  }

  public StartPos chooseStartPos() {
    switch(startPosChooser.getSelected()) {
      case LEFT_CS:
        return StartPos.LEFT_CS;
      case MID_CS:
        return StartPos.MID_CS;
      default:
        return StartPos.RIGHT_CS;
    }
  }


  public SendableChooser<AutoModes> getChooser() {
    return autoChooser;
  }
    

  private enum AutoModes {
    SIMPLE_DRIVE, SIMPLE_TRAJECTORY, AUTO_ALIGN_TRAJECTORY, PRIORITY_1_AUTO, PRIORITY_2_AUTO, PRIORITY_3_AUTO, PRIORITY_4_AUTO
  }

  private enum StartPos {
    LEFT_CS, MID_CS, RIGHT_CS
  }
}
