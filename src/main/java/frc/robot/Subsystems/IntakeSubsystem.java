package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.GamePiece;

public class IntakeSubsystem extends SubsystemBase {
    public static final int CONE_STATE = 0;
    public static final int CUBE_STATE = 1;

    private CANSparkMax intakeMotorInner;
    private CANSparkMax intakeMotorOuter;

    private int state;

    public int getState() {
        return state;
    }

    public void setState(int state) {
        this.state = state;
    }

    public IntakeSubsystem() {
        intakeMotorInner = new CANSparkMax(Constants.INTAKE_MOTOR_INNER, MotorType.kBrushless);
        intakeMotorOuter = new CANSparkMax(Constants.INTAKE_MOTOR_OUTER, MotorType.kBrushless);

        intakeMotorInner.setIdleMode(IdleMode.kBrake);
        intakeMotorInner.restoreFactoryDefaults();
        intakeMotorOuter.setIdleMode(IdleMode.kBrake);
        intakeMotorOuter.restoreFactoryDefaults();

        state = CUBE_STATE;
    }

    public void releaseGamePiece(double speedMultiplier) {
        if (state == CONE_STATE) {
            releaseCone(speedMultiplier);
        } else if (state == CUBE_STATE) {
            releaseCube(speedMultiplier);
        }
    }

    public void intakeGamePiece(double speedMultiplier) {
        if (state == CONE_STATE) {
            pickUpCone(speedMultiplier);
        } else if (state == CUBE_STATE) {
            pickUpCube(speedMultiplier);
        }
    }

    public void pickUpCone(double speedMultiplier) {
        intakeMotorInner.set(-speedMultiplier);
        intakeMotorOuter.set(-speedMultiplier);
    }

    public void releaseCone(double speedMultiplier) {
        intakeMotorInner.set(speedMultiplier);
        intakeMotorOuter.set(speedMultiplier);
    }

    public void pickUpCube(double speedMultiplier) {
        intakeMotorInner.set(speedMultiplier);
        intakeMotorOuter.set(-speedMultiplier);
    }

    public void releaseCube(double speedMultiplier) {
        intakeMotorInner.set(-speedMultiplier);
        intakeMotorOuter.set(speedMultiplier);
    }

    public void stopMotors() {
        intakeMotorInner.set(0);
        intakeMotorOuter.set(0);
    }

}
