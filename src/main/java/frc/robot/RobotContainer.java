// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotMap;
import frc.robot.PathPlanningCode.AutoUtils;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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

  
  public Command getAutonomousCommand() {
    return autoUtils.runPath(path);
  }

}