package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.vision.BaseVisionIO.BaseVisionIOInput;
import frc.robot.subsystems.vision.BaseVisionIO.vision_configuration_type;

import static frc.robot.Constants.Vision.ANGULAR_STDEV_COEFF;
import static frc.robot.Constants.Vision.ANGULAR_STDEV_MEGATAG_2_COEFF;
import static frc.robot.Constants.Vision.CAMERA_0_POS;
import static frc.robot.Constants.Vision.CAMERA_1_POS;
import static frc.robot.Constants.Vision.LINEAR_STDEV_COEFF;
import static frc.robot.Constants.Vision.LINEAR_STDEV_MEGATAG_2_COEFF;
import static frc.robot.Constants.Field.APRIL_TAG_LAYOUT;
import static frc.robot.Constants.Vision.MAX_UNCERTAINTY;
import static frc.robot.Constants.Vision.MAX_Z_ERROR;

import java.util.LinkedList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {

    public static Vision createVision(Drivetrain drivetrain) {
        if (RobotBase.isReal()) {
            return new Vision(drivetrain::addVisionMeasurement,
                    new PhotonVisionIO("photoncam-left", CAMERA_0_POS),
                    new PhotonVisionIO("photoncam-right", CAMERA_1_POS));
        } else {
            return new Vision(drivetrain::addVisionMeasurement,
                    new SimPhotonVisionIO("camera_0", drivetrain::getPose, Constants.Vision.CAMERA_0_POS),
                    new SimPhotonVisionIO("camera_1", drivetrain::getPose, Constants.Vision.CAMERA_1_POS));
        }
    }

    // empty but can hold an object that implements vision consumer
    private final vision_consumer consumer;
    // empty array that can accept any object implementing the interface
    private final BaseVisionIO[] IO_base;
    // AutoLogged auto generated class
    private final BaseVisionIOInput[] input;

    public final Field2d[] fields;

    // elipces means multiple objects of vision_IO_Base class can be passed in so
    // multiple cameras
    public Vision(vision_consumer consumer, BaseVisionIO... IO_base) {
        // this passes the private final consumer in
        // so running the method uses parameters to define private final variables which
        // then can't be changed
        this.consumer = consumer;
        this.IO_base = IO_base;

        this.input = new BaseVisionIOInput[IO_base.length];
        this.fields = new Field2d[IO_base.length];
        for (int i = 0; i < IO_base.length; i++) {
            input[i] = new BaseVisionIOInput();
            fields[i] = new Field2d();
            SmartDashboard.putData("Vision/camera" + i, fields[i]);
        }
    }

    // returns X angle to nearest tag, method
    public Rotation2d getAngleX(int cameraIndex) {
        return input[cameraIndex].angle_to_tag.rot_x();
    }

    // updates input and logs for each camera
    @Override
    public void periodic() {
        for (int i = 0; i < IO_base.length; i++) {
            IO_base[i].update_inputs(input[i]);
            // Logger.processInputs("Vision/Camera" + i, input[i]);
        }

        // does a bunch of suff for each camera
        for (int i = 0; i < IO_base.length; i++) {
            List<Pose3d> tag_poses = new LinkedList<>();
            List<Pose3d> robot_poses = new LinkedList<>();
            List<Pose3d> accepted_poses = new LinkedList<>();
            List<Pose3d> rejected_poses = new LinkedList<>();

            // each tag seen ID appends its position to tag poses
            for (int tag_ID : input[i].april_tag_IDs) {
                var tag_pose = APRIL_TAG_LAYOUT.getTagPose(tag_ID);
                if (tag_pose.isPresent()) {
                    tag_poses.add(tag_pose.get());
                }
            }

            for (var estimation : input[i].pose_estimation_data) {
                // confirms estimation data is with in thresholds
                // example has many unnessesary seeming conditions
                // come back to this because its a potential point of failure

                boolean tagCountInvalid = estimation.april_tag_count() == 0;
                boolean uncertaintyInvalid = (estimation.april_tag_count() == 1
                        && estimation.uncertainty() > MAX_UNCERTAINTY);
                boolean zErrorInvalid = Math.abs(estimation.position().getZ()) > MAX_Z_ERROR;
                boolean xOutBounds = (estimation.position().getX() < 0.0)
                        || (estimation.position().getX() > APRIL_TAG_LAYOUT.getFieldLength());
                boolean yOutBounds = (estimation.position().getY() < 0.0)
                        || (estimation.position().getY() > APRIL_TAG_LAYOUT.getFieldWidth());

                boolean reject_pose = tagCountInvalid || uncertaintyInvalid || zErrorInvalid || xOutBounds
                        || yOutBounds;

                robot_poses.add(estimation.position()); // stores all robot positions for a camera
                if (!reject_pose) {
                    accepted_poses.add(estimation.position()); // stores only accepted poses
                } else {
                    rejected_poses.add(estimation.position()); // stores onlt rejected poses
                }

                if (reject_pose) {
                    continue; // skips to next loop
                }

                // calculate stdev
                // stdev is an aproximation it is hueristic as the real thing can't be found
                // for some reason this baseline works with a multiplier to find x, y, and theta
                // stdevs
                double stdev_factor = Math.pow(estimation.average_tag_distance(), 2.0) / estimation.april_tag_count();
                double linear_stdev = LINEAR_STDEV_COEFF * stdev_factor;
                double angular_stdev = ANGULAR_STDEV_COEFF * stdev_factor;

                // MTag2 configs
                if (estimation.type() == vision_configuration_type.METATAG_2) {
                    linear_stdev *= LINEAR_STDEV_MEGATAG_2_COEFF;
                    angular_stdev *= ANGULAR_STDEV_MEGATAG_2_COEFF;
                }

                // Adds vision data to drivebase estimator
                consumer.accepts(
                        estimation.position().toPose2d(),
                        estimation.timestamp(),
                        VecBuilder.fill(linear_stdev, linear_stdev, angular_stdev));
            }

            // logs data by camera
            fields[i].getObject("EstimatedPoses").setPoses(robot_poses.stream().map(Pose3d::toPose2d).toList());
            fields[i].getObject("AcceptedPoses").setPoses(accepted_poses.stream().map(Pose3d::toPose2d).toList());
            fields[i].getObject("RejectedPoses").setPoses(rejected_poses.stream().map(Pose3d::toPose2d).toList());
        }
    }

    // marks a function interface
    // functional interface allows for 1 method here accepts()
    @FunctionalInterface
    // consumer accepts but returns nothing
    public static interface vision_consumer {
        public void accepts(
                Pose2d robot_pose,
                double timestamp,
                // how much uncertainty there is
                // <N3, N1> takes 3 standerd devaitions from vector
                Matrix<N3, N1> stdevs);
    }
}