// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotMap;
import frc.robot.PathPlanningCode.AutoUtils;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;

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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RobotContainer {
  //instantiate subsystems and commands
  private Joystick joystick = new Joystick(RobotMap.JOYSTICK_PORT1);

  private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem();
  private final PowerDistribution powerDistribution = new PowerDistribution();

  //auto stuff
  private String path = "Pathweaver/paths/exit1.path";
  private AutoUtils autoUtils = new AutoUtils(drivetrainSubsystem);



  public RobotContainer() {
    //register subsystems and set default commands
    powerDistribution.clearStickyFaults();
    
    CommandScheduler.getInstance().registerSubsystem(drivetrainSubsystem);
    drivetrainSubsystem.setDefaultCommand(new DriveCommand(drivetrainSubsystem,
      () -> -joystick.getRawAxis(0),
      () -> -joystick.getRawAxis(1)));

    configureButtonBindings();
  }


  private void configureButtonBindings() {

  }


  public DrivetrainSubsystem getDrive() {
    return drivetrainSubsystem;
  }

  public SequentialCommandGroup simpleRamseteConfig() {
        TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.DriveConstants.Autos.maxVelocity,
                Constants.DriveConstants.Autos.maxAccel)
            .setKinematics(Constants.DriveConstants.DRIVE_KINEMATICS);

      Trajectory traj = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(0)),
            config);
       RamseteCommand ramseteCommand =
        new RamseteCommand(
            traj,
            drivetrainSubsystem::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(
            Constants.DriveConstants.Autos.kS, 
            Constants.DriveConstants.Autos.kV, 
            Constants.DriveConstants.Autos.kA),
            Constants.DriveConstants.DRIVE_KINEMATICS,
            drivetrainSubsystem::getWheelSpeeds,
            new PIDController(0.1, 0, 0),
            new PIDController(0.1, 0, 0),
            drivetrainSubsystem::tankDriveVolts,
            drivetrainSubsystem);

    return ramseteCommand.andThen(() -> drivetrainSubsystem.tankDriveVolts(0, 0));
  }

  public RunCommand getSimpleCmdGrp() {
    return new RunCommand(() -> drivetrainSubsystem.testAutoDrive(0.5, 0.5), drivetrainSubsystem);
  }

  public Command getAutonomousCommand() {
    return autoUtils.runPath(path);
  }

}