package frc.robot.subsystems.vision;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

import static frc.robot.Constants.Field.APRIL_TAG_LAYOUT;

import java.util.HashSet;
import java.util.List;
import java.util.LinkedList;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;

public class PhotonVisionIO implements BaseVisionIO{
    protected final PhotonCamera camera;
    protected final Transform3d camera_pos;

    //constructor method for camera objects
    public PhotonVisionIO(String name, Transform3d camera_pos){
        camera = new PhotonCamera(name);
        this.camera_pos = camera_pos;
    }

    //nearly identical to Limelight reference limelight
    @Override 
    public void update_inputs(BaseVisionIOInput inputs){
        //instance variable tells if controller is found
        inputs.controller_found = camera.isConnected();

        Set<Short> april_tag_IDs = new HashSet<>();
        List<pose_estimation_data> pose_estimation_data = new LinkedList<>();

        for (var raw_data : camera.getAllUnreadResults()) {
            if (raw_data.hasTargets()){
                // sets instance variable to the angle to tag
                inputs.angle_to_tag = 
                    new rotation(
                        Rotation2d.fromDegrees(raw_data.getBestTarget().getPitch()), 
                        Rotation2d.fromDegrees(raw_data.getBestTarget().getYaw()));
            } else {
                // if no apriltags are found sets angle_to_tag to zero
                inputs.angle_to_tag = new rotation(Rotation2d.kZero, Rotation2d.kZero);
            }

            if (raw_data.multitagResult.isPresent() == true) { 
                //works like raw data in the event of multiple tags
                var multitagResult = raw_data.multitagResult.get();

                // calculate bot pose
                Transform3d feild_to_cam = multitagResult.estimatedPose.best;
                Transform3d feild_to_bot = feild_to_cam.plus(camera_pos.inverse());
                Pose3d bot_pos = new Pose3d(feild_to_bot.getTranslation(), feild_to_bot.getRotation());

                //calculate avg tag distance
                double average_tag_distance = 0.0;
                int i = 0;
                for (var tag : raw_data.targets) {
                    average_tag_distance += tag.bestCameraToTarget.getTranslation().getNorm();
                    i++;
                }
                average_tag_distance /= i;

                //adds apriltags 
                april_tag_IDs.addAll(multitagResult.fiducialIDsUsed);

                
                pose_estimation_data.add(
                    new pose_estimation_data(

                            raw_data.getTimestampSeconds(), 
                            
                            multitagResult.estimatedPose.ambiguity, 

                            multitagResult.fiducialIDsUsed.size(), 

                            average_tag_distance, 

                            bot_pos, 

                            vision_configuration_type.PHOTOVISION));

            } else if(!raw_data.targets.isEmpty()) { //single tag
                var tag = raw_data.targets.get(0);

                //calc bot pose
                var tag_pos = APRIL_TAG_LAYOUT.getTagPose(tag.fiducialId);
                if (tag_pos.isPresent()) {
                    Transform3d feild_to_tag =
                        new Transform3d(tag_pos.get().getTranslation(), tag_pos.get().getRotation());
                    Transform3d cam_to_tag = tag.bestCameraToTarget;
                    Transform3d feild_to_cam = feild_to_tag.plus(cam_to_tag.inverse());
                    Transform3d feild_to_bot = feild_to_cam.plus(camera_pos.inverse());
                    Pose3d bot_pos = new Pose3d(feild_to_bot.getTranslation(), feild_to_bot.getRotation());

                    //add tag IDs
                    april_tag_IDs.add((short) tag.fiducialId);

                    // add data
                    pose_estimation_data.add(
                    new pose_estimation_data(
                            raw_data.getTimestampSeconds(), 
                            tag.poseAmbiguity, 
                            1, 
                            cam_to_tag.getTranslation().getNorm(), 
                            bot_pos, 
                            vision_configuration_type.PHOTOVISION));
                }
            }
            
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
}