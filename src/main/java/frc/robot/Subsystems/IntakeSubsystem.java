package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax intakeMotor;

    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushless);

        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.enableVoltageCompensation(12);
    }

    public void pickUp() {
        intakeMotor.set(0.8);
    }

    public void release() {
        intakeMotor.set(-0.8);
    }
}
