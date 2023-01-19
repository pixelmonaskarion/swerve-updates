
import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.IOException;
import java.nio.file.FileSystems;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.PathPlanningCode.WaypointParser;

public class WaypointParserTest {
    private TrajectoryGenerator.ControlVectorList controlVectors = new TrajectoryGenerator.ControlVectorList();
    
    @Test
    public void getWaypoints() throws IOException { 
        //var expectedList = new ArrayList<List<Double>>();
        
        controlVectors.add(new Spline.ControlVector(new double[] {7.370389231190149, -0.6245939917920662}, new double[] {-3.542893461012311+8.1, 0.23011357592339232}));
        controlVectors.add(new Spline.ControlVector(new double[] {5.874650987688097, -0.8194487104192959}, new double[] {-4.430474396716827+8.1, -0.8743028395305534}));
        controlVectors.add(new Spline.ControlVector(new double[] {5.052816787961696, 0.18080352393980803}, new double[] {-6.2878196880984945+8.1, -0.7889608317373478}));
        
        var pathweaver_file = FileSystems.getDefault().getPath("src/main/deploy/Pathweaver/paths/exit1.path");
        TrajectoryGenerator.ControlVectorList parsedwaypoints = WaypointParser.getControlVectors(pathweaver_file);
        assertEquals(controlVectors, parsedwaypoints);
    }
}
