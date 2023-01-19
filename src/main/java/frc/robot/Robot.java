
package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
	// get instances of all subsystems (w/ a subsystem manager for many subsystems)

	private RobotContainer m_robotContainer;

	private Command m_autonomousCommand;
	private PhotonCamera camera;
	private PIDController controller = new PIDController(Constants.DriveConstants.kP, Constants.DriveConstants.kI, 0);
	private ProfiledPIDController profiledController = new ProfiledPIDController(Constants.DriveConstants.kP, 0, 0,  new TrapezoidProfile.Constraints(Constants.DriveConstants.Autos.maxVelocity, Constants.DriveConstants.Autos.maxAccel));

	private final double camHeight = 1; //change
	private final double targetHeight = 0.5; //change
	private final double tagGoalDist = 1.5;

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
		double forwardSpeed;
		double rotation;
		double range; 

		var result = camera.getLatestResult();
		boolean hasTargets = result.hasTargets();
		if (hasTargets) {
			
			PhotonTrackedTarget target = result.getBestTarget();
			double yaw = target.getYaw();
			int targetID = target.getFiducialId();
			System.out.println("got target id: " + targetID);

			range  = PhotonUtils.calculateDistanceToTargetMeters(camHeight, targetHeight, 0, Units.degreesToRadians(result.getBestTarget().getPitch()));
			rotation = -profiledController.calculate(yaw, 0);
			forwardSpeed  = -controller.calculate(range, tagGoalDist);
		} else {
			//System.out.println("no targets found!");
			forwardSpeed = 0;
			rotation = 0;
			range  = 0;
		}

		if (Math.abs(range-tagGoalDist) > 1 || Math.abs(rotation) > 5) {
			m_robotContainer.getDrive().drive(rotation, forwardSpeed, 0.5, Constants.JoystickScaling.LINEAR, Constants.DriveStyle.ARCADE_TANK);
		} else {
			m_robotContainer.getDrive().tankDriveVolts(0, 0);
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
