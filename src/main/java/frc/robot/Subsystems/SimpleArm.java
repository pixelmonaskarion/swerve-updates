package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SimpleArm extends SubsystemBase {
    private CANSparkMax armMotor;
    private boolean status;

    private Timer armTimer;
    
    public SimpleArm() {
        armMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);
        armMotor.setIdleMode(IdleMode.kBrake);

        armTimer = new Timer();
        status = true;
    }

    public void moveArm(int buttonNum) {
        if(buttonNum == 6) {
            raise();
            if (!status) {
                armTimer.reset();
            }
            status = true;
        } else if (buttonNum == 7) {
            lower();
            if (status) {
                armTimer.reset();
            }
            status = false;
        } else {
            armMotor.set(0);
        }
    }
    
    public void raise() {
        if(armTimer.get() < 2){
            armMotor.set(0.3);
        } else {
            armMotor.set(0.05);
        }
    }

    public void lower() {
        if(armTimer.get() < 2){
            armMotor.set(-0.3);
        } else {
            armMotor.set(-0.05);
        }
    }
   
}

