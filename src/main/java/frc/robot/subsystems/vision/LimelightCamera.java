package frc.robot.subsystems.vision;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.function.Supplier;
import java.util.Set;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

//import java.util.function.Supplier;
//import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightCamera implements CameraInterface {

    // tracks if rewind capture taken
    private boolean captured = false;
    // private boolean been_enabled = false; for an on disable situation

    // supliers store functions so they are more like variables
    private final Supplier<Rotation2d> rotation_supplier;


    // publisher sends data in/on a topic which works like a channel subscriber on
    // same topic receives it
    private final DoubleArrayPublisher orientation_publisher;

    private final NetworkTableEntry rewind_entry;

    private final DoubleSubscriber latency_subscriber;
    private final DoubleSubscriber rot_x_subscriber;
    private final DoubleSubscriber rot_y_subscriber;

    private final DoubleArraySubscriber hardwareSubscriber;

    private final DoubleArraySubscriber metatag1Subscriber;
    private final DoubleArraySubscriber metatag2Subscriber;

    public LimelightCamera(String name, Supplier<Rotation2d> rotation_supplier) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(name);

        // enables rewind to play back footage
        table.getEntry("rewind_enable_set").setDouble(0);

        this.rotation_supplier = rotation_supplier;
        //https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
        //string keys are already defined by limelight see above
        orientation_publisher = table.getDoubleArrayTopic("robot_orientation_set").publish();

        rewind_entry = table.getEntry("capture_rewind");

        latency_subscriber = table.getDoubleTopic("tl").subscribe(0.0); //these exact strings must be used
        rot_x_subscriber = table.getDoubleTopic("tx").subscribe(0.0);
        rot_y_subscriber = table.getDoubleTopic("ty").subscribe(0.0);

        hardwareSubscriber = table.getDoubleArrayTopic("hw").subscribe(new double[] {});

        metatag1Subscriber = table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[] {});
        metatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
    }

    private boolean been_enabled = false;
    // overrides default method
    @Override
    public void update_inputs(BaseVisionIOInput inputs) {

        double matchTime = Timer.getMatchTime();
        
        if (DriverStation.isEnabled()) {
            been_enabled = true;
        }
        

        //rwnd
        if (DriverStation.isDisabled() && !captured && matchTime < 2 && been_enabled) {
            captured = true;
            double[] currentArray = rewind_entry.getDoubleArray(new double[] {});
            double counter = (currentArray.length > 0) ? currentArray[0] : 0;

            double[] setter = {++counter, VisionConstants.MAX_REWIND_TIME};

            // each time counter (index 0) is incremented a capture is taken
            rewind_entry.setDoubleArray(setter);
        }

        // checks camera connection based of off if there was an update in the last 250 ms
        inputs.cam_connected = ((RobotController.getFPGATime() - latency_subscriber.getLastChange()) / 1000) < 250; 

        // updates angle to tag
        inputs.angle_to_tag = 
            new rotation(
                Rotation2d.fromDegrees(rot_x_subscriber.get()), 
                Rotation2d.fromDegrees(rot_y_subscriber.get()));

        //publisher sends to network table
        //.get gets stored function
        orientation_publisher.accept(new double[] {rotation_supplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
        
        // updates network table
        NetworkTableInstance.getDefault().flush();

        // Set is like an unordered array without duplicates
        Set<Integer> april_tag_IDs = new HashSet<>();
        // in a linked list each index is called in order and leads to the next
        List<pose_estimation_data> pose_estimation_data = new LinkedList<>();

        //hardware metrics taken 
        double[] hw = hardwareSubscriber.get();
        if (hw.length == 4) {
            inputs.temp = hw[0];
            inputs.fps = hw[3];
        }

        // for each bit of raw data that has changed since the last call
        for (var raw_data : metatag1Subscriber.readQueue()) {
            //System.out.println(raw_data);
            // Java short-hand
            if (raw_data.value.length == 0) continue;
            // 11 is that tags start with
            //read this
            //https://github.com/LimelightVision/limelightlib-wpijava/blob/main/LimelightHelpers.java
            // lines 700-800
            for (int i = 11; i < raw_data.value.length; i+=7) { 
                // adds each tag ID as tag IDs start at 11 and increment by 7
                april_tag_IDs.add((int) raw_data.value[i]);
            }

            // .add() appends to list
            pose_estimation_data.add(
                    new pose_estimation_data(

                            (raw_data.timestamp * 1.0e-6)-(raw_data.value[6] * 1.0e-3) , //raw_data.value[6] is latency
                            
                            // if there is more than one tag takes a non zero ambiguity
                            raw_data.value.length >= 18 ? raw_data.value[17] : 0.0, 

                            (int) raw_data.value[7], // tag count

                            raw_data.value[9], // avg tag distance

                            parse(raw_data.value), //pos

                            vision_configuration_type.METATAG_1));

        }

        for (var raw_data : metatag2Subscriber.readQueue()) {
            //System.out.println(raw_data);
            // Java short-hand
            if (raw_data.value.length == 0) continue;
            // 11 is that tags start with
            //read this
            //https://github.com/LimelightVision/limelightlib-wpijava/blob/main/LimelightHelpers.java
            // lines 700-800
            for (int i = 11; i < raw_data.value.length; i+=7) { 
                //TODO document here
                april_tag_IDs.add((int) raw_data.value[i]);
            }

            // .add() appends to list
            pose_estimation_data.add(
                    new pose_estimation_data(

                            (raw_data.timestamp * 1.0e-6)-(raw_data.value[6] * 1.0e-3) , //raw_data.value[6] is latency
                            
                            // inhereintly certain
                            0.0, 

                            (int) raw_data.value[7], // tag count

                            raw_data.value[9], // avg tag distance

                            parse(raw_data.value), // pos

                            vision_configuration_type.METATAG_2));

        }

        // saves estimation data to objects
        inputs.pose_estimation_data = new pose_estimation_data[pose_estimation_data.size()];
        for (int i = 0; i < pose_estimation_data.size(); i++) {
            inputs.pose_estimation_data[i] = pose_estimation_data.get(i);
        }

        // saves april tag IDs to objects
        // creates new array where the number of elements is the number of tags
        inputs.april_tag_IDs = new int[april_tag_IDs.size()];
        int i = 0;
        for (int ID : april_tag_IDs) {
            inputs.april_tag_IDs[i++] = ID;
        }
    }
    
    private static Pose3d parse(double limelight_array[]){
        return new Pose3d(
            limelight_array[0],
            limelight_array[1],
            limelight_array[2],
            new Rotation3d(
                Units.degreesToRadians(limelight_array[3]),
                Units.degreesToRadians(limelight_array[4]),
                Units.degreesToRadians(limelight_array[5])
            )
        );
    }
    
}