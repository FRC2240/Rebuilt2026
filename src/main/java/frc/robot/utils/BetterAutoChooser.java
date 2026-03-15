package frc.robot.utils;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
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
    
    private SendableChooser<String> chooserA = new SendableChooser<>();
    private SendableChooser<String> chooserB = new SendableChooser<>();
    private SendableChooser<String> chooserC = new SendableChooser<>();

    public BetterAutoChooser(SendableChooser<String> A, SendableChooser<String> B, SendableChooser<String> C) {
        //this.chooserA = A;
        //this.chooserB = B;
        //this.chooserC = C;
    }

    private static List<String> getAllPathNames() {
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

    public void publishPathChoosers() {
        List<String> pathNames = getAllPathNames();

        for (String name : pathNames) {
            this.chooserA.addOption(name, name);
            this.chooserB.addOption(name, name);
            this.chooserC.addOption(name, name);
        }

        this.chooserA.addOption("none", "none");
            this.chooserB.addOption("none", "none");
            this.chooserC.addOption("none", "none");

        SmartDashboard.putData("Path Selector A", this.chooserA);
        SmartDashboard.putData("Path Selector B", this.chooserB);
        SmartDashboard.putData("Path Selector C", this.chooserC);
    }

    public Command buildAuto(Supplier<Command> shootCommand, Drivetrain drive, int waitTime) {
        PathPlannerPath pathA;
        PathPlannerPath pathB;
        PathPlannerPath pathC;

        try{
            pathA = PathPlannerPath.fromPathFile(this.chooserA.getSelected());
            pathB = PathPlannerPath.fromPathFile(this.chooserB.getSelected());
            pathC = PathPlannerPath.fromPathFile(this.chooserC.getSelected());

        } catch (Exception e) {
            DriverStation.reportWarning("Path could not be initialized for building:\n" + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }

        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                var startPose = pathA.getStartingHolonomicPose();
                drive.resetPose(startPose.get());
            }),  // End of 1st action

            AutoBuilder.followPath(pathA), // End of 2nd action

            new ParallelRaceGroup(shootCommand.get(), new WaitCommand(waitTime)).andThen(Commands.print("hello")), // End of 3rd action

            AutoBuilder.followPath(pathB),

            new ParallelRaceGroup(shootCommand.get()).raceWith(new WaitCommand(waitTime)),

            AutoBuilder.followPath(pathC),

            shootCommand.get()

            );
    }
    
}
