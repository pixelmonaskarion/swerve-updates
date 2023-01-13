package frc.robot.subsystems;

import static java.util.Objects.requireNonNull;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveStyle;
import frc.robot.Constants.JoystickScaling;

public class DrivetrainOptions extends RobotDriveBase {
    private final MotorControllerGroup leftMotors;
    private final MotorControllerGroup rightMotors;
    private double leftSpeed;
    private double rightSpeed;
    private static double scaleFactorA = 0.4;
    private static double scaleFactorB = 0.55;
    private static double scaleFactorC = 0.35;
    private static double scaleFactorD = 0.6;

   
    public DrivetrainOptions(MotorControllerGroup leftMotors, MotorControllerGroup rightMotors) {
        requireNonNull(leftMotors, "leftmotors could not be null");
        requireNonNull(rightMotors, "rightmotors could not be null");
        this.leftMotors = leftMotors;
        this.rightMotors = rightMotors;
        
    }

    @Override
    public void stopMotor() {
        leftMotors.stopMotor();
        rightMotors.stopMotor();
        feed();
    }

    @Override
    public String getDescription() {
        return "Drivetrain Options";
    }

    //set left and right motor controller group speeds for each joystick scaling type
    public void pickDrivetrain(double turnSpeed, double forwardSpeed, JoystickScaling scaleType, double curveScaling, DriveStyle driveStyle) {
        double turnPower = scaleValue(turnSpeed, scaleType);
        double forwardPower = scaleValue(forwardSpeed, scaleType);

        switch(driveStyle) {
            case CUSTOM_TANK:
                rightSpeed = (turnPower -(-forwardPower-(turnPower/curveScaling)))/(1+Math.abs(turnPower/curveScaling));
                leftSpeed =(turnPower +(-forwardPower+(turnPower/curveScaling)))/(1+Math.abs(turnPower/curveScaling));

               
                rightSpeed = MathUtil.clamp(rightSpeed, -1, 1);
                leftSpeed = MathUtil.clamp(leftSpeed,-1,1);

                leftMotors.set(leftSpeed);
                rightMotors.set(rightSpeed);

                break;
            case NORMAL_ARCADE:
                leftSpeed = scaleFactorA*Math.abs(turnSpeed)+scaleFactorB*turnSpeed*turnSpeed;
                rightSpeed = scaleFactorC*Math.abs(forwardSpeed)+scaleFactorD*forwardSpeed*forwardSpeed;

                leftMotors.set(leftSpeed);
                rightMotors.set(rightSpeed);
                
                break;
            case ARCADE_TANK:
                leftSpeed = scaleFactorA*Math.abs(turnSpeed)+scaleFactorB*turnSpeed*turnSpeed;
                rightSpeed = scaleFactorC*Math.abs(forwardSpeed)+scaleFactorD*forwardSpeed*forwardSpeed;
                
                if (turnSpeed < 0) {
                    leftSpeed *= -1;
                }

                if (forwardSpeed < 0) {
                    rightSpeed *= -1;
                }

                leftMotors.set(leftSpeed - rightSpeed);
                rightMotors.set(rightSpeed + leftSpeed);
        
                break;
            case SATURATED_ARCADE:
                double saturatedInput;
                double greaterInput = Math.max(Math.abs(turnSpeed), Math.abs(forwardSpeed));
                double smallerInput = Math.min(Math.abs(turnSpeed), Math.abs(forwardSpeed));

                if (greaterInput > 0.0) {
                    saturatedInput = smallerInput/greaterInput + 1;
                } else {
                    saturatedInput = 1.0;
                }

                turnPower = turnPower/saturatedInput;
                forwardPower = forwardPower/saturatedInput;

                rightSpeed = forwardPower+turnPower;
                leftSpeed = turnPower-forwardPower;

                leftMotors.set(leftSpeed);
                rightMotors.set(rightSpeed);

                break;
            default:
                rightSpeed = turnPower-forwardPower;
                leftSpeed = turnPower+forwardPower;

                leftMotors.set(leftSpeed);
                rightMotors.set(rightSpeed);
        }
            
    }

    private double scaleValue(double rawInput, JoystickScaling scaleType) {
        double scaledValue = rawInput;
        scaledValue = MathUtil.applyDeadband(scaledValue, DriveConstants.DEFAULT_DEADBAND);

        if (scaleType == JoystickScaling.SQUARED_EXPONENTIAL) {
            scaledValue *= Math.abs(scaledValue);
        } else if (scaleType ==JoystickScaling.CUBIC_EXPONENTIAL) {
            scaledValue *= scaledValue*scaledValue;
        } else if (scaleType == JoystickScaling.SQUARE_ROOTED) {
            scaledValue = Math.sqrt(Math.abs(scaledValue));
        } else if (scaleType == JoystickScaling.CUBE_ROOTED) {
            scaledValue = Math.cbrt(Math.abs(scaledValue));
        } else if (scaleType == JoystickScaling.LOGARITHMIC) {
            scaledValue = (scaledValue != 0) ? (Math.log(Math.abs(scaledValue))/Math.log(Math.E)) : 0;
        }

        MathUtil.clamp(scaledValue, -1, 1); //sets bounds for scaledValue
        return scaledValue;
    }
    
}
