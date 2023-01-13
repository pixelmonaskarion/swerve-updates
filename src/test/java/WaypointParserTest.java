import static org.junit.Assert.assertEquals;

import java.io.IOException;

import org.junit.Test;

import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.PathPlanningCode.WaypointParser;

public class WaypointParserTest {
    private TrajectoryGenerator.ControlVectorList controlVectors = new TrajectoryGenerator.ControlVectorList();
    
    @Test
    public void getWaypoints() throws IOException { 
    controlVectors.add(new Spline.ControlVector(new double[] {7.370389231190149, -0.6245939917920662}, new double[] {-3.542893461012311+8.1, 0.23011357592339232}));
    controlVectors.add(new Spline.ControlVector(new double[] {5.874650987688097, -0.8194487104192959}, new double[] {-4.430474396716827+8.1, -0.8743028395305534}));
    controlVectors.add(new Spline.ControlVector(new double[] {5.052816787961696, 0.18080352393980803}, new double[] {-6.2878196880984945+8.1, -0.7889608317373478}));

    assertEquals(WaypointParser.getControlVectors(Filesystem.getDeployDirectory().toPath().resolve("/src/main/deploy/Pathweaver/paths/exit1.path")), controlVectors);
    }
}
