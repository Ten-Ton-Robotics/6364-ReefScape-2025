// package frc.robot.autonomous;

// import java.util.Optional;

// import org.photonvision.EstimatedRobotPose;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.util.PhotonVisionHandler;

// public class ReefAlign {

//     private final CommandSwerveDrivetrain m_drivetrain; // the drivetrain to move
//     private final PhotonVisionHandler m_PhotonVisionHandler;
//     // private final ProfiledPIDController m_angleController; // profiled PID controller for the angle
//     // private final ProfiledPIDController m_xController; // PID controller for the X position of the
//     //                                                     // robot
//     // private final ProfiledPIDController m_yController; // PID controller for the Y position of the
//     //                                                  // robot

//     private Optional<EstimatedRobotPose> prevVisionOut = Optional.empty();
//     private Optional<EstimatedRobotPose> VisionOut;

 
//     public ReefAlign(CommandSwerveDrivetrain drivetrain, PhotonVisionHandler photonVisionHandler){
         
//         m_drivetrain = drivetrain;
//         m_PhotonVisionHandler = photonVisionHandler;

//         // m_angleController = new ProfiledPIDController(5, 0, 0, new Constraints(1, 2));
//         // m_xController = new ProfiledPIDController(5, 0, 0, new Constraints(1, 2));
//         // m_yController = new ProfiledPIDController(5, 0, 0, new Constraints(1, 2));
//     }

//     public Optional<Pose2d> getPose(){
//         if(VisionOut.isPresent()){
//             return Optional.of(VisionOut.get().estimatedPose.toPose2d());
//         } else{
//             return Optional.empty();
//         }
//     }    

//     public void updatePose(){

//         if (prevVisionOut.isPresent()) {
//             VisionOut = m_PhotonVisionHandler.getEstimatedGlobalPose(prevVisionOut.get().estimatedPose.toPose2d(), true);
//         } else {
//             VisionOut = m_PhotonVisionHandler.getEstimatedGlobalPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)), true);
//         }

//         prevVisionOut = VisionOut;

//     }
// }
