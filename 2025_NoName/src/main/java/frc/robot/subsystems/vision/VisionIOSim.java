package frc.robot.subsystems.vision;

import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOSim extends VisionIOReal {
    private static VisionSystemSim visionSim;
    private final PhotonCameraSim simCamera;
    private final Supplier<Pose2d> poseSupplier;


  /**
   * Creates a new VisionIOSim interfacing the VisionIOReal
   *
   * @param The VisionConstants of the camera.
   * @param The Pose2d supplier for vision odometry.
   */
    public VisionIOSim(VisionConstants constants, Supplier<Pose2d> poseSupplier) {
        super(constants);
        this.poseSupplier = poseSupplier;

        // Initialize visionSim if still Null
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(VisionConstants.aprilTagLayout);
        }
        // Set Camera Properties to Simulate Real Cameras
        var cameraProp = new SimCameraProperties();

        // This is the 3D calibration resolution, highest resolution = most accurate poses
        cameraProp.setCalibration(
            1200, 
            800,
            Rotation2d.fromDegrees(70));
        cameraProp.setCalibError(0.0, 0.0);
        /**
         * This is framerate of our cameras when they run the calibrated resolution. 
         * Its in a general range so I decided to just set it to 50 since it averages around there.
        */ 
        cameraProp.setFPS(50.0); 
        /**
         * This is latency, hard to replicate the latency but 30 miliseconds is a good estimate.
        */ 
        cameraProp.setAvgLatencyMs(30.0);
        cameraProp.setLatencyStdDevMs(5.0);
        // apply given properties to camera
        simCamera = new PhotonCameraSim(camera, cameraProp);
        /**
         * Turns on the wireframes. which useful for debug website when running Simulation, 
         * shows wireframes around all the tags visible to easily see what the cameras exactly see.
        */
        simCamera.enableDrawWireframe(true);
        // Sets the max range of visibility, needed otherwise cameras can detect targets very far in simulation
        simCamera.setMaxSightRange(7.0);

        visionSim.addCamera(simCamera, robotToCamera);
    }

    // Updates the Inputs and Feeds back to VisionIOReal, while updating the visionSim pose Supplier for Vision Odometry. 
    @Override
    public void updateInputs() {
        visionSim.update(poseSupplier.get());
        super.updateInputs();
    }

    // Returns name of the Camera
    @Override
    public String getName(){
        return camera.getName();
    }

    // Returns given VisionConstants from Instanization
    @Override
    public VisionConstants getVisionConstants(){
      return constants;
    }
}
