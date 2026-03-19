package frc.robot.utils.BetterAutos;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class BetterAutoChooser extends SubsystemBase{

    private SendableChooser<Translation2d> startChooser = new SendableChooser<>();
    private SendableChooser<PathPlannerPath> collectionOneChooser = new SendableChooser<>();
    private SendableChooser<PathPlannerPath> collectionTwoChooser = new SendableChooser<>();

    private List<PathPlannerPath> pathList = new ArrayList<>();
    private List<NamedCommandSupplier> commandList = new ArrayList<>();

    public BetterAutoChooser() {
    }

    public void registerCommand(String name, Supplier<Command> command) {
        commandList.add(new NamedCommandSupplier(name, command));
    }


    private void getAllPaths() {
        List<String> pathNames = new ArrayList<>();

        File pathDir = new File(Filesystem.getDeployDirectory(), "pathplanner/paths");

        File[] files = pathDir.listFiles((dir, name) -> name.endsWith("OTF.path"));

        if (files != null) {
            for (File file : files) {

                pathNames.add(file.getName().replace(".path", ""));
            }
        }

        // this should make it so all paths on the pathList are valid, and thus all selected values will be valid
        try {
            if (pathNames.size() == 0) {return;}

            for (String path : pathNames) {
                pathList.add(PathPlannerPath.fromPathFile(path));
                System.out.println(path);
            }
        } catch (Exception e) {
            DriverStation.reportWarning("Failed to load all paths from file:\n" + e.getMessage(), e.getStackTrace());
        }
    }

    private void initializeChoosers () {
        getAllPaths();

        startChooser.setDefaultOption("None", null);
        collectionOneChooser.setDefaultOption("None", null);
        collectionTwoChooser.setDefaultOption("None", null);

        startChooser.addOption("Hub", AutoConstants.HUB.get());
        startChooser.addOption("Left Bump", AutoConstants.LEFT_BUMP.get());
        startChooser.addOption("Left Trench", AutoConstants.LEFT_TRENCH.get());
        startChooser.addOption("Right Bump", AutoConstants.RIGHT_BUMP.get());
        startChooser.addOption("Left Trench", AutoConstants.RIGHT_TRENCH.get());

        for (PathPlannerPath path : pathList) {
            collectionOneChooser.addOption(path.name, path);
            collectionTwoChooser.addOption(path.name, path);
        }
    }

    /* deffered for after the rest works
    @Override
    public void periodic() {
        collectionOneChooser.onChange((newValue) -> {
            // feed it into tree function to get a new list of valid paths2 and publish that to collection chooser 2
            updateChoosers(newValue);
        });
    }

    private void updateChoosers(PathPlannerPath selected) {
        if (selected == null) {return;}
        boolean changed = false;
        for (PathPlannerPath path : pathList) {

            if (selected.name.contains("Left") && !path.name.contains("Right") && !path.name.contains("Outpost")) {
                changed = true;

                collectionTwoChooser = new SendableChooser<>();
                collectionTwoChooser.addOption(path.name, path);
            }
            else if (selected.name.contains("Right") && !path.name.contains("Left") && !path.name.contains("Depot")) {
                changed = true;

                collectionTwoChooser = new SendableChooser<>();
                collectionTwoChooser.addOption(path.name, path);
            }

        }

        if (changed) {
            publishChoosers();
        }
    }
    */

    public void publishChoosers() {
        initializeChoosers();

        SmartDashboard.putData("BetterChooser/Start Selector", startChooser);
        SmartDashboard.putData("BetterChooser/Collection 1 Selector", collectionOneChooser);
        SmartDashboard.putData("BetterChooser/Collection 2 Selector", collectionTwoChooser);

    }

    public Command[] initilizeAuto(Drivetrain drivetrain) {
        Command[] autoCommands = new Command[5];

        Supplier<Command> defaultCommand = commandList.get(0).GetCommand();
        Supplier<Command> shootCommand = commandList.get(1).GetCommand();

        Translation2d startTrans2d = startChooser.getSelected();
        PathPlannerPath pathA = collectionOneChooser.getSelected();
        PathPlannerPath pathB = collectionTwoChooser.getSelected();

        // This gets your initial position and 
        autoCommands[0] = new InstantCommand(() -> {    // this is not meant to use drivetrain, but the rotation at the paths start
                var start = Optional.of(new Pose2d(startTrans2d, drivetrain.getHeading()));
                drivetrain.resetPose(start.get());
            });
        
        if (pathA == null) {
            autoCommands[1] = Commands.none();
        }
        else {
            autoCommands[1] = new ParallelRaceGroup(AutoBuilder.followPath(pathA))
                        .raceWith(defaultCommand.get());
        }

        autoCommands[2] = new ParallelRaceGroup(shootCommand.get()).raceWith(new WaitCommand(AutoConstants.WAITTIME));

        if (pathA == null) {
            autoCommands[3] = Commands.none();
        }
        else {
            System.out.println("|" + pathB.name + "|");
            autoCommands[3] = new ParallelRaceGroup(AutoBuilder.followPath(pathB))
                        .raceWith(defaultCommand.get());
        }

        autoCommands[4] = shootCommand.get();

        return autoCommands;
    }

    public Command buildAuto(Drivetrain drivetrain) {
        Command[] auto = initilizeAuto(drivetrain);
        return new SequentialCommandGroup(auto);
    }
}
