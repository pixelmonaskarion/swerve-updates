package frc.robot.commands;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PathPlanningCode.AutoUtils;
import frc.robot.subsystems.DrivetrainSubsystem;

//for list of objs in CodeTeamTraining path list, pick one randomly
public class RandomTrajectoryGeneratorCommand extends CommandBase {
    private String fileLocation;
    private DrivetrainSubsystem drivetrainSubsystem;
    private AutoUtils autoUtils = new AutoUtils(drivetrainSubsystem);
    
    public RandomTrajectoryGeneratorCommand(DrivetrainSubsystem drivetrainSubsystem, String fileLocation) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.fileLocation = fileLocation;
    }

    private static List<String> getListOfPaths(String fileLocation) throws IOException {
        List<String> paths = new ArrayList<>();
        BufferedReader reader = new BufferedReader(new FileReader(fileLocation));

        String line;
        while ((line = reader.readLine()) != null) {
            paths.add(line);
        }
        reader.close();
        return paths;
    }

    private String getRandomPath() {
        try {
            List<String> paths = getListOfPaths(fileLocation);
            Random random = new Random();
            int index = random.nextInt(paths.size());
            String randomPath = paths.get(index);
            
            return randomPath;
        } catch (IOException e) {
            e.printStackTrace();
            return null;
        }
    }

    @Override
    public void execute() {;
        autoUtils.runPath(getRandomPath());
    }

    @Override
    public void end(boolean interrupted) {
        autoUtils.runPath(getRandomPath()).cancel();
    }
}
