package frc.robot.utils.BetterAutos;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.utils.*;

public class BetterAutoChooser extends SubsystemBase{

    private SendableChooser<PathPlannerPath> collectionOneChooser = new SendableChooser<>();
    private SendableChooser<PathPlannerPath> collectionTwoChooser = new SendableChooser<>();

    private List<PathPlannerPath> pathList = new ArrayList<>();
    private List<NamedCommandSupplier> commandList = new ArrayList<>();

    private boolean collectionChanged = false;

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
        for (String path : pathNames) {
            try {
                if (pathNames.size() == 0) {return;}

                pathList.add(PathPlannerPath.fromPathFile(path));
            } catch (Exception e) {
                DriverStation.reportWarning("Failed to load all paths from file:\n" + e.getMessage(), e.getStackTrace());
            }
        }
    }

    private void initializeChoosers() {
        getAllPaths();

        collectionOneChooser.setDefaultOption("None", null);
        collectionTwoChooser.setDefaultOption("None", null);

        collectionOneChooser.onChange((s) -> this.collectionChanged = true);

        for (PathPlannerPath path : pathList) {
            collectionOneChooser.addOption(path.name, path);
            collectionTwoChooser.addOption(path.name, path);
        }
    }

    private void updateChoosers() {
        collectionTwoChooser = new SendableChooser<>();

        collectionOneChooser.setDefaultOption("None", null);
        collectionTwoChooser.setDefaultOption("None", null);

        List<PathPlannerPath> pList = new ArrayList<>();

        for (PathPlannerPath path : pathList) {
            if(collectionOneChooser.getSelected() == null) {
                pList.add(path);
                continue;
            }

            String pathName = path.name.toLowerCase();
            String pathSelect = collectionOneChooser.getSelected().name.toLowerCase();

            if(collectionChanged) {
                if(pathSelect.startsWith("left") && !(pathName.startsWith("right") || pathName.endsWith("outpost otf"))) {
                        pList.add(path);
                }

                else if(pathSelect.startsWith("right") && !(pathName.startsWith("left") || pathName.endsWith("depot otf"))) {
                        pList.add(path);
                }
            }
            else {
                pList.add(path);
            }
        }

        for (PathPlannerPath path : pList) {
            collectionTwoChooser.addOption(path.name, path);
        }

        SmartDashboard.putData("BetterChooser/Collection 1 Selector", collectionOneChooser);
        SmartDashboard.putData("BetterChooser/Collection 2 Selector", collectionTwoChooser);
    }


    public void publishChoosers() {
        initializeChoosers();

        SmartDashboard.putData("BetterChooser/Collection 1 Selector", collectionOneChooser);
        SmartDashboard.putData("BetterChooser/Collection 2 Selector", collectionTwoChooser);

    }

    public Command[] initilizeAuto(Drivetrain drivetrain) {
        Command[] autoCommands = new Command[5];

        Supplier<Command> defaultCommand = commandList.get(0).GetCommand();
        Supplier<Command> shootCommand = commandList.get(1).GetCommand();

        PathPlannerPath pathA = collectionOneChooser.getSelected();
        PathPlannerPath pathB = collectionTwoChooser.getSelected();

        // Returns PathA unless null, then PathB, but if both are null it returns null
        PathPlannerPath startPath = (pathA != null) ? pathA : ((pathB != null) ? pathB : null);
        if (startPath == null) { // Catch's null from prev line
            return new Command[] {Commands.none()};
        }

        autoCommands[0] = new InstantCommand(() -> {
            if (DriverStation.getAlliance().get() == Alliance.Red) {
                var start = startPath.flipPath().getStartingHolonomicPose();
                RobotPosition.reset(start.get());
            }
            else {
                var start = startPath.getStartingHolonomicPose();
                RobotPosition.reset(start.get());
            }
        }); 

        if (pathA == null) {
            autoCommands[1] = Commands.none();

            autoCommands[2] = Commands.none();
        }
        else {
            autoCommands[1] = new ParallelRaceGroup(AutoBuilder.followPath(pathA)).raceWith(defaultCommand.get());
            
            autoCommands[2] = new ParallelRaceGroup(shootCommand.get()).raceWith(new WaitCommand(AutoConstants.WAITTIME));
        }

        if (pathB == null) {
            autoCommands[3] = Commands.none();
        }
        else {
            autoCommands[3] = new ParallelRaceGroup(AutoBuilder.followPath(pathB)).raceWith(defaultCommand.get());
        }

        autoCommands[4] = shootCommand.get();

        return autoCommands;
    }

    public Command buildAuto(Drivetrain drivetrain) {
        Command[] auto = initilizeAuto(drivetrain);
        return new SequentialCommandGroup(auto);
    }

    @Override
    public void periodic() {
        if(collectionChanged) {
            updateChoosers();
            collectionChanged = false;
        }
    }
}
