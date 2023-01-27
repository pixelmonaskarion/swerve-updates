package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private CANSparkMax armMotor;
    //uses trapezoidal motion profile to reduce jittering, note spark max encoders have units revs/sec
    private ProfiledPIDController controller;
    private ArmFeedforward armFeedforward = new ArmFeedforward(0.05, 1.09, 1.95);
   
    private boolean enabled;

    public Arm(double initialGoalPosition) {
        armMotor = new CANSparkMax(Constants.ARM_MOTOR_ID, MotorType.kBrushless);

        controller =  new ProfiledPIDController(Constants.kP, 0, 0, 
        new TrapezoidProfile.Constraints(0.4, 0.5));

        //on initialization move arm down on neutral position
        setGoal(initialGoalPosition);
    }

    @Override
    public void periodic() {
        if (enabled) {
            //set motor voltage using controller-calculated feedforward
            useOutput(getMeasurement(), controller.getSetpoint());
        }
    }

    //motion profile goal in radians
    //because the arm will always have 2 possible states, the goal will be the same distance, different sign
    public void setGoal(double goal) {
        setGoal(new TrapezoidProfile.State(goal, 0)); //end location (rev) and velocity (rev/sec)
    }

    public void setGoal(TrapezoidProfile.State goalState) {
        controller.setGoal(goalState);
    }

    public double getMeasurement() {
        return armMotor.getEncoder().getPosition();
    }

    public void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);
        armMotor.set(output + feedforward);
    }

    public void enable() {
        enabled = true;
        controller.reset(getMeasurement());
    }

    public void disable() {
        enabled = false;
    }
    
    public boolean isEnabled() {
        return isEnabled();
    }


    public ProfiledPIDController getController() {
        return controller;
    }
}
