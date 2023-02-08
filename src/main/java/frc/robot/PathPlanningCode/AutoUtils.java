
package frc.robot.PathPlanningCode;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.DriveSubsystem;

public class AutoUtils {
    private SendableChooser<AutoModes> autoChooser = new SendableChooser<>();
    
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
