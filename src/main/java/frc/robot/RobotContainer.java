
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Commands.VisionTurnCommand;
import frc.robot.Commands.VisionTranslateCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.PathPlanningCode.AutoUtils;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive;
  //private final VisionSubsystem m_vision;
  private final VisionSubsystem m_vision;

  private final AutoUtils autoUtils = new AutoUtils();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    
    //SmartDashboard.putBoolean("BButton pressed", getController().getBButton());


    m_robotDrive = new DriveSubsystem();
    //m_vision = new VisionSubsystem();
    m_vision = new VisionSubsystem();

    configureButtonBindings();
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-m_driverController.getLeftY(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getLeftX(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getRightX(), 0.06),
                MathUtil.applyDeadband(-m_driverController.getRightY(), 0.06),
                m_driverController.getRightBumper(), m_driverController.getAButton()),
            m_robotDrive));

  }


  private void configureButtonBindings() {

      new Trigger(() -> m_driverController.getBButton())
      .whileTrue(new VisionTurnCommand(m_vision, m_robotDrive, m_driverController));
      
   
      new Trigger(() -> m_driverController.getYButton())
      .onTrue(new VisionTranslateCommand(m_vision, m_robotDrive, m_driverController));

    

  }


  public XboxController getController() {
    return m_driverController;
  }

  public DriveSubsystem getDrive() {
    return m_robotDrive;
  }
 
  public VisionSubsystem getVision() {
    return m_vision;
  }


  public AutoUtils getAutoRoutine() {
    return autoUtils;
  }
}