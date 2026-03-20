package frc.robot.utils;

import java.io.File;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoCreator extends SubsystemBase {
    Supplier<Command> shootCommandSupplier;
    Consumer<Pose2d> resetPoseConsumer;

    SendableChooser<String> startPointChooser;
    boolean startPointChanged = false;
    boolean collectionChanged = false;
    SendableChooser<String> collection1Chooser;
    SendableChooser<String> collection2Chooser;

    public AutoCreator(Consumer<Pose2d> resetPoseConsumer, Supplier<Command> shootCommandSupplier) {
        this.shootCommandSupplier = shootCommandSupplier;
        this.resetPoseConsumer = resetPoseConsumer;

        startPointChooser = new SendableChooser<>();

        startPointChooser.setDefaultOption("None", null); // No auto at all
        startPointChooser.addOption("Left Trench", "left_trench");
        startPointChooser.addOption("Left Bump", "left_bump");
        startPointChooser.addOption("Hub", "hub");
        startPointChooser.addOption("Right Bump", "right_bump");
        startPointChooser.addOption("Right Trench", "right_trench");

        startPointChooser.onChange((s) -> this.startPointChanged = true);
        SmartDashboard.putData("Auto Creator/Start Point", startPointChooser);

        createCollection1Chooser();
        createCollection2Chooser();
    }

    @Override
    public void periodic() {
        if (startPointChanged) {
            String before = collection1Chooser.getSelected();
            createCollection1Chooser();
            if ((before == null && collection1Chooser.getSelected() == null)
                    || !before.equals(collection1Chooser.getSelected())) {
                collectionChanged = true;
            }
            startPointChanged = false;
        }

        if (collectionChanged) {
            createCollection2Chooser();
            collectionChanged = false;
        }
    }

    private void createCollection1Chooser() {
        String startPoint = startPointChooser.getSelected();
        collection1Chooser = new SendableChooser<>();

        // None does not exist on this chooser unless there is no auto or it is a hub
        // auto
        if (startPoint == null || startPoint.equals("hub")) {
            collection1Chooser.setDefaultOption("None", null);
        }

        if (startPoint != null) {
            if (!startPoint.startsWith("hub")) {
                collection1Chooser.addOption("Neutral Zone Pickup", "neutral");
            }

            if (startPoint.startsWith("left")) {
                collection1Chooser.addOption("Depot Pickup", "depot");
            }

            if (startPoint.startsWith("right")) {
                collection1Chooser.addOption("Outpost Pickup", "outpost");
            }
        }
        collection1Chooser.onChange((s) -> this.collectionChanged = true);

        SmartDashboard.putData("Auto Creator/Collection 1", collection1Chooser);
    }

    private void createCollection2Chooser() {
        String startPoint = startPointChooser.getSelected();
        String path1 = collection1Chooser.getSelected();
        collection2Chooser = new SendableChooser<>();
        collection2Chooser.setDefaultOption("None", null);

        if (startPoint != null && path1 != null) {
            System.out.println(startPoint);
            System.out.println(path1);
            if (!startPoint.startsWith("hub")) {
                collection2Chooser.addOption("Neutral Zone Pickup", "neutral");
            }

            if (startPoint.startsWith("left") && !path1.equals("depot")) {
                collection2Chooser.addOption("Depot Pickup", "depot");
            }

            if (startPoint.startsWith("right") && !path1.equals("outpost")) {
                collection2Chooser.addOption("Outpost Pickup", "outpost");
            }
        }

        System.out.println("anotha");
        SmartDashboard.putData("Auto Creator/Collection 2", collection2Chooser);

    }

    public PathPlannerPath[] getPaths() {
        File pathDirectory = new File(Filesystem.getDeployDirectory().toPath() + "/pathplanner/paths");
        File[] filePaths = pathDirectory.listFiles((file, name) -> name.startsWith("creator_"));
        PathPlannerPath[] paths = new PathPlannerPath[filePaths.length];

        for (int i = 0; i < filePaths.length; i++) {
            try {
                String name = filePaths[i].getName();
                int end = name.lastIndexOf(".path");
                paths[i] = PathPlannerPath.fromPathFile(filePaths[i].getName().substring(0, end));
            } catch (Exception e) {
                System.out.println("EXCEPTION");
                System.out.println(e);
                return null;
            }
        }
        return paths;
    }

    public Command getCommand() {
        try {
            String startPoint = startPointChooser.getSelected();

            // No start point means no auto
            if (startPoint == null)
                return Commands.none();

            String side = null;
            if (startPoint.startsWith("left"))
                side = "left";
            if (startPoint.startsWith("right"))
                side = "right";
            String collection1 = collection1Chooser.getSelected();
            String collection2 = collection2Chooser.getSelected();

            Command driveCollection1Command = Commands.none();
            ;

            if (startPoint.equals("hub")) {
                PathPlannerPath path = PathPlannerPath.fromPathFile("creator_hub");
                Pose2d startingPose = AllianceRelativePose2d.fromBluePose(path.getStartingHolonomicPose().get()).get();
                return Commands.sequence(
                        Commands.runOnce(() -> resetPoseConsumer.accept(startingPose)),
                        AutoBuilder.followPath(path),
                        shootCommandSupplier.get());
            } else if (collection1 == null) {
                // If there is no collection and the start is not hub, do nothing
                return Commands.none();
            }

            if (collection1.equals("outpost")) {
                // Outpost has two different paths to run in one go
                PathPlannerPath path1 = PathPlannerPath.fromPathFile("creator_" + startPoint + "_outpost");
                PathPlannerPath path2 = PathPlannerPath.fromPathFile("creator_outpost_shoot");
                Pose2d startingPose = AllianceRelativePose2d.fromBluePose(path1.getStartingHolonomicPose().get()).get();

                driveCollection1Command = Commands.sequence(
                        Commands.runOnce(() -> resetPoseConsumer.accept(startingPose)),
                        AutoBuilder.followPath(path1),
                        Commands.waitSeconds(2),
                        AutoBuilder.followPath(path2));
            } else {
                PathPlannerPath path = PathPlannerPath.fromPathFile("creator_" + startPoint + "_neutral");
                Pose2d startingPose = AllianceRelativePose2d.fromBluePose(path.getStartingHolonomicPose().get()).get();

                driveCollection1Command = Commands.sequence(
                        Commands.runOnce(() -> resetPoseConsumer.accept(startingPose)),
                        AutoBuilder.followPath(path));
            }

            // No second leg specified, run the first collection and then shoot
            if (collection2 == null) {
                return driveCollection1Command.andThen(shootCommandSupplier.get());
                        //shootCommandSupplier.get());
            }

            System.out.println(startPoint + "_" + collection1);
            System.out.println(side + "_" + collection2 + "_2");

            return Commands.none();

        } catch (Exception e) {
            System.out.println("AUTO CREATOR ERROR");
            System.out.println(e);
        }

        return Commands.none();
    }

}
