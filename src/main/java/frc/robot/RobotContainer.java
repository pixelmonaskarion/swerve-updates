
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.VisionTurnCommand;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.OIConstants;
import frc.robot.Commands.ArmTest;
import frc.robot.Commands.ElevatorCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.ScoreGamePieceCommand;
import frc.robot.Commands.SimpleDriveCommand;
import frc.robot.Commands.ToStartConfigCommand;
import frc.robot.Commands.VisionTranslateCommand;
import frc.robot.PathPlanningCode.AutoUtils;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem m_robotDrive;
  private final VisionSubsystem m_vision;
  private final ElevatorSubsystem m_elevator;
  private final ArmSubsystem m_arm;
  private final IntakeSubsystem m_intake;

  private double intakeSpeedMultiplier;

  private final AutoUtils autoUtils = new AutoUtils();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  Joystick m_operatorController = new Joystick(OIConstants.kIOperatorControllerPort);


  public RobotContainer() {
    SmartDashboard.putBoolean("GamePiece/CubeMode", false);
    SmartDashboard.putBoolean("GamePiece/ConeMode", false);
   
    m_robotDrive = new DriveSubsystem();
    m_vision = new VisionSubsystem();
    m_elevator = new ElevatorSubsystem();
    m_arm = new ArmSubsystem();
    m_intake = new IntakeSubsystem();

    intakeSpeedMultiplier = wAxisSpeedMultiplier();

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
    
    m_elevator.setDefaultCommand(
        new RunCommand(
          () -> m_elevator.simpleMovement(
            m_operatorController.getRawAxis(1)), m_elevator));

  }


  private void configureButtonBindings() {

    new Trigger(() -> m_driverController.getBButton())
      .whileTrue(new VisionTurnCommand(m_vision, m_robotDrive, m_driverController));
   
    new Trigger(() -> m_driverController.getYButton())
      .onTrue(new VisionTranslateCommand(m_vision, m_robotDrive, m_driverController));

    new Trigger(() -> triggerPressed())
      .whileTrue(new SimpleDriveCommand(m_robotDrive, m_driverController));

    new Trigger(() -> m_operatorController.getRawButton(1))
        .whileTrue(new InstantCommand(() -> m_intake.releaseGamePiece(intakeSpeedMultiplier), m_intake));

      new Trigger(() -> m_operatorController.getRawButton(2))
      .whileTrue(new InstantCommand(() -> m_intake.intakeGamePiece(intakeSpeedMultiplier), m_intake));

    new Trigger(() -> m_operatorController.getRawButton(14))
      .onTrue(new InstantCommand(() -> m_intake.stopMotors(), m_intake));

    //not periodic
    if (m_operatorController.getRawButton(4)) {
      Constants.curGamePiece = GamePiece.CONE;
    }

    if (m_operatorController.getRawButton(3)) {
      Constants.curGamePiece = GamePiece.CUBE;
    }
    
    new Trigger(() -> m_operatorController.getRawButton(16))
      .onTrue(new InstantCommand(m_arm::retract));
       
    new Trigger(() -> m_operatorController.getRawButton(15))
      .onTrue(new InstantCommand(m_arm::expand));
  }

  
  public double wAxisSpeedMultiplier() {
    double mult = (m_operatorController.getRawAxis(3) + 1)/2;
    return MathUtil.clamp(Math.log(mult*10), 0.1, 0.8);
  }

  public boolean triggerPressed() {
    if (m_driverController.getLeftTriggerAxis() != 0 || m_driverController.getRightTriggerAxis() != 0) {
      return true;
    }
    return false;
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

  public ElevatorSubsystem getElevator() {
    return m_elevator;
  }

  public ArmSubsystem getArm() {
    return m_arm;
  }

  public IntakeSubsystem getIntake() {
    return m_intake;
  }

  public AutoUtils getAutoUtils() {
    return autoUtils;
  }
}
