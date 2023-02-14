package frc.robot.Subsystems;



import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {
    private Solenoid sol1;
    private Solenoid sol2;
    private Solenoid sol3;
    private Solenoid sol4;
    private Solenoid[] sols;

    public ArmSubsystem() {
        sol1 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_P1);
        sol2 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_P2);
        sol3 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_P3);
        sol4 = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.SOLENOID_P4);

        sols = new Solenoid[]{sol1, sol2, sol3, sol4};
        for (Solenoid sol : sols) {
            sol.setPulseDuration(0.4);
        }
    }


    public void expand() {
        for (Solenoid sol : sols) {
            sol.set(true);
        }
    }

    public void retract() {
        for (Solenoid sol : sols) {
            sol.set(false);
        }
    }

}