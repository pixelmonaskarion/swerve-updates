// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/**
 * This is a demo program showing the use of the DifferentialDrive class, specifically it contains
 * the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive m_myRobot;
  private Joystick stick;

  private final CANSparkMax m_leftMotor_front = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax m_leftMotor_back = new CANSparkMax(2, MotorType.kBrushless);
  private final CANSparkMax  m_rightMotor_front = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax  m_rightMotor_back = new CANSparkMax(4, MotorType.kBrushless);
  private final MotorControllerGroup left = new MotorControllerGroup(m_leftMotor_front,m_leftMotor_back);
  private final MotorControllerGroup right = new MotorControllerGroup(m_rightMotor_front,m_rightMotor_back);
  
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    right.setInverted(true);

    m_myRobot = new DifferentialDrive(left, right);
    stick = new Joystick(0);
  }

  @Override
  public void teleopPeriodic() {
    m_myRobot.arcadeDrive(-stick.getY(), -stick.getX());
  }
}
