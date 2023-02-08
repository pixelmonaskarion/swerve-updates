
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
import frc.robot.Subsystems.DriveSubsystem;

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

    public Command simpleCmdGrp(DriveSubsystem drivetrain) {
        return new RunCommand(() -> drivetrain.drive(0, 0, 0, 0, false, false), drivetrain).withTimeout(5);
    }

    

  


  public Command chooseAuto(DriveSubsystem drivetrain) {
    switch(autoChooser.getSelected()) {
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
