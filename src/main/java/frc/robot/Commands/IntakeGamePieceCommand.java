package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.ElevatorSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;

public class IntakeGamePieceCommand extends CommandBase {
    private ElevatorSubsystem elevator;
    private ArmSubsystem arm;
    private IntakeSubsystem intake;

    private final Timer timer;
    private static final double SETPOINT = -2;
    
    //retract elevator, arm down, turn on intake
    public IntakeGamePieceCommand(ElevatorSubsystem elevator, ArmSubsystem arm, IntakeSubsystem intake) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;

        timer = new Timer();
        addRequirements(elevator, arm, intake);
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().schedule(new ElevatorCommand(elevator, SETPOINT));
        arm.retract();

        timer.start();
    }

    @Override
    public void execute() {
        intake.pickUp();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3) ? true : false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }

}
