package frc.robot.PathPlanningCode;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class AutoUtils {
    private SendableChooser<AutoModes> autoChooser = new SendableChooser<>();
    private GenerateTrajectoryFromFile trajectory = new GenerateTrajectoryFromFile();
    private String test_path = "Pathweaver/paths/exit1.path";

    public AutoUtils() {
        autoChooser.setDefaultOption("Simple Drive", AutoModes.SIMPLE_DRIVE);
        autoChooser.addOption("Simple Ramsete", AutoModes.SIMPLE_RAMSETE);
        autoChooser.addOption("Custom Ramsete", AutoModes.CUSTOM_RAMSETE);

        SmartDashboard.putData("auto choices", autoChooser);
    }

    public Command simpleCmdGrp(Drivetrain drivetrain) {
        return new RunCommand(() -> drivetrain.testDrive(0.1, 0.1), drivetrain).withTimeout(5);
    }

    //to do: add velocity and acceleration constraints
    public Command simpleRamseteConfig(Drivetrain drivetrain) {
        TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.MAX_VELOCITY,
                Constants.MAX_ACCEL)
            .setKinematics(Constants.DRIVE_KINEMATICS);

      Trajectory traj = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(0)),
            config);

       RamseteCommand ramseteCommand =
        new RamseteCommand(
            traj,
            drivetrain::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(
                Constants.kS, 
                Constants.kV, 
                Constants.kA),
            Constants.DRIVE_KINEMATICS,
            drivetrain::getWheelSpeeds,
            new PIDController(0.1, 0, 0),
            new PIDController(0.1, 0, 0),
            drivetrain::tankDriveVolts,
            drivetrain);
        
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }

  public Command customRamseteCommand(Drivetrain drivetrain, String file_path) {

    RamseteCommand ramseteCommand = 
      new RamseteCommand(trajectory.generateTrajectory(
        file_path, 
        GenerateTrajectoryFromFile.createTrajConfig(
          Constants.MAX_VELOCITY, 
          Constants.MAX_ACCEL, 
          0, 
          0, 
          new CentripetalAccelerationConstraint(4))), 
      drivetrain::getPose, new RamseteController(), 
      new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA),
      Constants.DRIVE_KINEMATICS, drivetrain::getWheelSpeeds, 
      new PIDController(Constants.kD, Constants.kI, 0),
      new PIDController(Constants.kD, Constants.kI, 0),
      drivetrain::tankDriveVolts, drivetrain);
      
      drivetrain.resetEncoders();
        //stop robot after autonomous trajectory is followed
    return ramseteCommand.andThen(() -> drivetrain.tankDriveVolts(0, 0));
  }


  public Command chooseAuto(Drivetrain drivetrain) {
    switch(autoChooser.getSelected()) {
        case SIMPLE_RAMSETE:
            return simpleRamseteConfig(drivetrain);
        case CUSTOM_RAMSETE:
            return customRamseteCommand(drivetrain, test_path);
        default:
            return simpleCmdGrp(drivetrain);
    }
  }

  public SendableChooser<AutoModes> getChooser() {
    return autoChooser;
  }
    

    private enum AutoModes {
        SIMPLE_DRIVE, SIMPLE_RAMSETE, CUSTOM_RAMSETE
    }
}
