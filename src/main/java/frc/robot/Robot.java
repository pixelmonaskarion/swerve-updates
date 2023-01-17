
package frc.robot;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	// get instances of all subsystems (w/ a subsystem manager for many subsystems)

	private RobotContainer m_robotContainer;

	private Command m_autonomousCommand;
	private PhotonCamera camera;

	// leave constructor empty for now, unless something needs to be done every time
	// robot is called
	public Robot() {

	}

	@Override
	public void robotInit() {
		m_robotContainer = new RobotContainer();
		// register subsystems into a list to minimize redundancy if logging
		// inputs/outputs for all
		camera = new PhotonCamera("USB2.0_PC_CAMERA");
	}

	@Override
	public void robotPeriodic() {
		// continue to run commands after initialization
		CommandScheduler.getInstance().run();
		// perform additional periodic calculations that don't fit inside the available
		// commands
		// e.g., pose calculations, outputting logs to smart dashboard, outputting
		// telemetry
		var result = camera.getLatestResult();
		boolean hasTargets = result.hasTargets();
		if (hasTargets) {
			List<PhotonTrackedTarget> targets = result.getTargets();
			PhotonTrackedTarget target = result.getBestTarget();
			double yaw = target.getYaw();
			double pitch = target.getPitch();
			double area = target.getArea();
			double skew = target.getSkew();
			int targetID = target.getFiducialId();
			double poseAmbiguity = target.getPoseAmbiguity();
			Transform3d bestCameraToTarget = target.getBestCameraToTarget();
			Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
			System.out.println("got target id: " + targetID);
		} else {
			//System.out.println("no targets found!");
		}
	}

	@Override
	public void disabledInit() {
		CommandScheduler.getInstance().cancelAll();
		m_robotContainer.getDrive().setBreakStatus(true);
	}

	// e.g., turn off coprocessor stuff
	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
		m_robotContainer.getDrive().setBreakStatus(false);
	}

	// initialize auto code (commands are periodically scheduled in robotPeriodic)
	@Override
	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();

		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	// schedule commands for what the robot should do after auto period ends
	@Override
	public void teleopInit() {

		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}

		m_robotContainer.getDrive().setBreakStatus(false);
	}

}
