package frc.robot.subsystems.vision;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.utils.Field;

import java.util.function.Supplier;

public class SimPhoton extends PhotonVisionCamera {

    private static VisionSystemSim vision_sim;
    
    private final PhotonCameraSim camera_sim;

    private final Supplier<Pose2d> pose_supplier;


    public SimPhoton(String name, Supplier<Pose2d> pose_supplier, Transform3d camera_pos){
        super(name, camera_pos);
        this.pose_supplier = pose_supplier;

        //
        if (vision_sim == null) {
            vision_sim = new VisionSystemSim("null");
            vision_sim.addAprilTags(Field.APRIL_TAG_LAYOUT);
        }
        //add sim camera
        var camera_properties = new SimCameraProperties();
        camera_sim = new PhotonCameraSim(camera, camera_properties, Field.APRIL_TAG_LAYOUT);
        vision_sim.addCamera(camera_sim, camera_pos);
    }
    
    @Override
    public void update_inputs(BaseVisionIOInput input){
        vision_sim.update(pose_supplier.get());
        super.update_inputs(input);
    }
}
