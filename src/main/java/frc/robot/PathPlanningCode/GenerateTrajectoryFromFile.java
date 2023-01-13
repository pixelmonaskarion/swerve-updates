package frc.robot.PathPlanningCode;

import java.nio.file.Path;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;


//resolves file path and gets control vectors from waypoints
public class GenerateTrajectoryFromFile {

    public Trajectory generateTrajectory(String file_path, TrajectoryConfig config) {

        Path traj_path = Filesystem.getDeployDirectory().toPath().resolve(file_path);
        TrajectoryGenerator.ControlVectorList controlVectors = WaypointParser.getControlVectors(traj_path);
    
        return TrajectoryGenerator.generateTrajectory(controlVectors, config);
       
    }

    public static TrajectoryConfig createTrajConfig(double maxSpeed, double maxAccel, double initSpeed, double finalSpeed, TrajectoryConstraint... constraints) {
        TrajectoryConfig config = new TrajectoryConfig(maxSpeed, maxAccel);

        config.setStartVelocity(initSpeed);
        config.setEndVelocity(finalSpeed);
        config.setKinematics(new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH));
        for (TrajectoryConstraint constraint : constraints) {
            config.addConstraint(constraint);
        }

        return config;
    }

    //basic trajectory constraints

}