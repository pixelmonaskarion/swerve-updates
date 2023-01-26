package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
	private Command m_autonomousCommand;

  public Robot() {

  }

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    /*m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}*/
    Command newAuto = m_robotContainer.getAutoPath().chooseAuto(m_robotContainer.getRobotDrive());
    if (newAuto != null) {
      newAuto.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    m_robotContainer.getRobotDrive().updateOdometry();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
