// package frc.robot.util;

// import java.io.UncheckedIOException;
// import java.util.ArrayList;
// import java.util.List;
// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.targeting.PhotonPipelineResult;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class PhotonVisionHandler2 {
//     private PhotonCamera frontCamera;
//     private PhotonCamera backCamera;
//     private AprilTagFieldLayout aprilTagFieldLayout;
//     private PhotonPoseEstimator frontPoseEstimator;
//     private PhotonPoseEstimator backPoseEstimator;

//     private final Transform3d robotToFrontCam =
//         new Transform3d(new Translation3d(Units.inchesToMeters(10.4), Units.inchesToMeters(-6.5),
//             Units.inchesToMeters(13.8)), new Rotation3d(0, Math.toRadians(-15), 0));

//     private final Transform3d robotToBackCam =
//         new Transform3d(new Translation3d(Units.inchesToMeters(-10.4), Units.inchesToMeters(6.5),
//             Units.inchesToMeters(13.8)), new Rotation3d(0, Math.toRadians(15), Math.PI));

//     public PhotonVisionHandler2() {
//         frontCamera = new PhotonCamera("Front");
//         backCamera = new PhotonCamera("Back");

//         try {
//             aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
//         } catch (UncheckedIOException e) {
//             System.err.println("Error loading AprilTag field layout: " + e.getMessage());
//             aprilTagFieldLayout = null;
//         }

//         frontPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
//             PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera, robotToFrontCam);
        
//         backPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
//             PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, backCamera, robotToBackCam);
//     }

//     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
//         frontPoseEstimator.setReferencePose(prevEstimatedRobotPose);
//         backPoseEstimator.setReferencePose(prevEstimatedRobotPose);

//         Optional<EstimatedRobotPose> frontPose = frontPoseEstimator.update(frontCamera.getLatestResult());
//         Optional<EstimatedRobotPose> backPose = backPoseEstimator.update(backCamera.getLatestResult());

//         if (frontPose.isPresent() && backPose.isPresent()) {
//             return Optional.of(mergePoses(frontPose.get(), backPose.get()));
//         } else if (frontPose.isPresent()) {
//             return frontPose;
//         } else {
//             return backPose;
//         }
//     }

//     import edu.wpi.first.math.geometry.Rotation2d;

//     private EstimatedRobotPose mergePoses(EstimatedRobotPose frontPose, EstimatedRobotPose backPose) {
//         Pose2d avgPose = new Pose2d(
//             (frontPose.estimatedPose.getX() + backPose.estimatedPose.getX()) / 2.0,
//             (frontPose.estimatedPose.getY() + backPose.estimatedPose.getY()) / 2.0,
//             new Rotation2d(
//                 (frontPose.estimatedPose.getRotation().getZ() + backPose.estimatedPose.getRotation().getZ()) / 2.0
//             )
//         );
//         return new EstimatedRobotPose(avgPose, frontPose.timestamp);
//     }
    
//     public ArrayList<Integer> getAprilTagIds() {
//         var results = frontCamera.getLatestResult();
//         var results2 = backCamera.getLatestResult();
//         ArrayList<Integer> ids = new ArrayList<>();

//         extractTagIds(results, ids);
//         extractTagIds(results2, ids);
        
//         return ids;
//     }

//     private void extractTagIds(PhotonPipelineResult result, ArrayList<Integer> ids) {
//         if (result.hasTargets()) {
//             for (var target : result.getTargets()) {
//                 ids.add(target.getFiducialId());
//             }
//         }
//     }

//     public double avgTagArea(List<PhotonTrackedTarget> tags) {
//         return tags.isEmpty() ? 0.0 : tags.stream().mapToDouble(PhotonTrackedTarget::getArea).average().orElse(0.0);
//     }
// }