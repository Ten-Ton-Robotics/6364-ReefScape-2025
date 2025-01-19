package frc.robot.util;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class PhotonVisionHandler {

  // private PhotonCameraSim cameraSim;
  // private VisionSystemSim visionSim;
  private PhotonCamera vision;
  private AprilTagFieldLayout aprilTagFieldLayout;
  // private boolean simulated;
  private PhotonPoseEstimator photonPoseEstimator;


  //Camera offset to center of robot including rotation
  private final Transform3d robotToCam =
      new Transform3d(new Translation3d(Units.inchesToMeters(10.4), Units.inchesToMeters(-6.5),
          Units.inchesToMeters(13.8)), new Rotation3d(0, Math.toRadians(-15), 0)); // Adjusted
                                                                                   // camera angle

  public PhotonVisionHandler() {

    // init camera
    vision = new PhotonCamera("Camera_Module_v1");

    // simulated = Utils.isSimulation();


    // Load AprilTag field layout
    try {
      aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); //TODO update to 2025 game
    } catch (Exception e) {
      System.err.println("Error loading AprilTag field layout: " + e.getMessage());
      aprilTagFieldLayout = null;
    }

    // Kalman filter to fuse vision measurements
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCam);

    // if (simulated) {
    //   initializeSimulation();
    // }
  }

  // private void initializeSimulation() {
  //   if (aprilTagFieldLayout == null) {
  //     System.out.println("AprilTag field layout is null. Simulation will not include tags.");
  //     return;
  //   }

  //   visionSim = new VisionSystemSim("main");
  //   visionSim.addAprilTags(aprilTagFieldLayout);

  //   var cameraProps = new SimCameraProperties();
  //   cameraProps.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
  //   cameraProps.setCalibError(0.35, 0.10);
  //   cameraProps.setFPS(120);
  //   cameraProps.setAvgLatencyMs(50);
  //   cameraProps.setLatencyStdDevMs(15);
  //   // cameraProps.setFOV(75); // Set appropriate FOV for your camera
  //   cameraProps.setAvgLatencyMs(11.0); // Based on actual PhotonVision latency
  //   cameraProps.setLatencyStdDevMs(3.0);

  //   cameraSim = new PhotonCameraSim(vision, cameraProps);
  //   visionSim.addCamera(cameraSim, robotToCam);

  //   cameraSim.enableDrawWireframe(true);
  //   System.out.println("PhotonVision simulation initialized");
  // }

  // public void updateSimulation(Pose2d robotPose) {
  //   if (simulated && visionSim != null) {
  //     Pose3d pose3d = new Pose3d(robotPose.getX(), robotPose.getY(), 0.0,
  //         new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians()));
  //     visionSim.update(robotPose);
  //   }
  // }

  // I'm not sure what the latency was for because I didn't document anything and it got depricated in the library - Jadyn
  
  // public double getLatencySeconds() {
  //   var result = vision.getLatestResult();
  //   return result.getLatencyMillis() / 1000.0;
  // }

  // public int getNumberOfTags() {
  //   var result = vision.getLatestResult();
  //   return result.hasTargets() ? result.getTargets().size() : 0;
  // }

  // public int getAprilTagID() {
  //   var result = vision.getLatestResult();
  //   PhotonTrackedTarget target = result.getBestTarget();
  //   return (target != null) ? target.getFiducialId() : -1;
  // }

  // public double areaOfAprilTag() {
  //   var result = vision.getLatestResult();
  //   PhotonTrackedTarget target = result.getBestTarget();
  //   return (target != null) ? target.getArea() : 0.0;
  // }


  // Method to get the estimated PhotonPose from the PV Kalman filter
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    boolean run = false;

    

    // Get the first result from the unread results
    if(vision.getAllUnreadResults().size() > 0){
      PhotonPipelineResult output = vision.getAllUnreadResults().get(0);

      if (!run) { // If run is false
        run = true;
        return photonPoseEstimator.update(output); // Update and return result
      }

      // Set the reference pose and update the estimator
      photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
      return photonPoseEstimator.update(output);

    }
    else{
      return Optional.empty();
    }
    }
  
}


  // public double avgTagArea(List<PhotonTrackedTarget> tags) {
  //   double temparea = 0;

  //   for (int i = 0; i < tags.size(); i++) {
  //     temparea += tags.get(i).getArea();
  //   }

  //   temparea = temparea / tags.size();

  //   return temparea;
  // }

  // public OptionalDouble getLatestLatencyAdjustedTimeStamp() {

  //   if (this.m_poseSubscriber == null) {

  //     return OptionalDouble.empty();

  //   } else {

  //     final TimestampedDoubleArray internal2 = m_poseSubscriber.getAtomic();
  //     return OptionalDouble.of((internal2.timestamp - internal2.value[6]) * 0.001);

  //   }

  // }
