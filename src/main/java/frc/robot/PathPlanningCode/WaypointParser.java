package frc.robot.PathPlanningCode;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

//puts waypoints for all quintic splines into list of control vectors
public class WaypointParser {
    public static TrajectoryGenerator.ControlVectorList getControlVectors(Path file_path) {
        TrajectoryGenerator.ControlVectorList controlVectors = new TrajectoryGenerator.ControlVectorList();
        final double chassisPathweaverOffset = 8.1;

        try (BufferedReader reader = new BufferedReader(new FileReader(file_path.toFile()))) {
            String contents = reader.readLine();
            boolean firstLine = false;

            while (contents != null) {
                if (!firstLine || !contents.contains(",")) {
                    firstLine = true;
                    contents = reader.readLine();
                    continue;
                }

                String[] parts = contents.split(",");
                //x, tangent x and y, tangent y
                controlVectors.add(new Spline.ControlVector(new double[] {Double.parseDouble(parts[0]),Double.parseDouble(parts[2])},
                new double[] {Double.parseDouble(parts[1])+chassisPathweaverOffset, Double.parseDouble(parts[3])}));

            }
        } catch (IOException e) {
            System.out.println("Path file parse error in file: " + file_path);
        }

        return controlVectors;
    }
}
