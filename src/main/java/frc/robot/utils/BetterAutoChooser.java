package frc.robot.utils;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class BetterAutoChooser {
    private final String none = "none";

    private List<SendableChooser<String>> chooserList = new ArrayList<>();
    private List<NamedCommand> commandList = new ArrayList<>();

    public BetterAutoChooser(int numChoosers) {
        for (int i = 0; i < numChoosers; i++) {
            chooserList.add(new SendableChooser<>());
        }
    }

    public void registerCommand(String name, Supplier<Command> command) {
        commandList.add(new NamedCommand(name, command));
    }

    private List<String> getAllPathNames() {
        List<String> pathNames = new ArrayList<>();

        File pathDir = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");

        File[] files = pathDir.listFiles((dir, name) -> name.endsWith(".path"));

        if (files != null) {
            for (File file : files) {

                pathNames.add(file.getName().replace(".path", ""));
            }
        }

        return pathNames;
    }

    public void publishChoosers() {
        List<String> pathNames = getAllPathNames();

        for (int i = 0; i < chooserList.size(); i++) {
            if(i % 2 == 0) {
                for (String name : pathNames) {
                    chooserList.get(i).addOption(name, name);
                }
            }
            else if (commandList.size() != 0) {
                for (NamedCommand command : commandList) {
                    chooserList.get(i).addOption(command.GetName(), command.GetName());
                }
            }

            chooserList.get(i).addOption(none, none);
            SmartDashboard.putData("OTF/Selector " + (i + 1), chooserList.get(i));
        }
    }

    public Command[] initializePaths(Drivetrain drive, double time) {
        Command[] pathCommands = new Command[chooserList.size() + 1];
        
        try {
            PathPlannerPath startPath = PathPlannerPath.fromPathFile(chooserList.get(0).getSelected());
            pathCommands[0] = new InstantCommand(() -> {
                var start = startPath.getStartingHolonomicPose();
                drive.resetPose(start.get());
            });
        } catch (Exception e) {
            DriverStation.reportError("Fatal Path Init Error:\n" + e.getMessage(), e.getStackTrace());
        }
        // index 0 should be set to reset command

        for (int i = 1; i < chooserList.size() + 1; i++) {
            if (i % 2 == 1) {
                try{
                    PathPlannerPath path = PathPlannerPath.fromPathFile(chooserList.get(i-1).getSelected());
                    pathCommands[i] = AutoBuilder.followPath(path);

                } catch (Exception e) {
                    DriverStation.reportWarning("Path could not be initialized:\n" + e.getMessage(), e.getStackTrace());
                    pathCommands[i] = Commands.none();
                }
            }
            else {
                for (NamedCommand command : commandList) {
                    if(command.GetName().equals(chooserList.get(i-1).getSelected())) {
                        pathCommands[i] = new ParallelRaceGroup(command.GetCommand().get()).raceWith(new WaitCommand(time));
                    }
                }
            }
        }
        return pathCommands;
    }

    public Command buildAuto(Drivetrain drive, double waitTime) {
        Command[] pathCommands = initializePaths(drive, waitTime);

        return new SequentialCommandGroup(pathCommands);
    }
    
}
