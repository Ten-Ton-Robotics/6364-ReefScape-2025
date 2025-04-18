package frc.robot.subsystems;

import java.util.List;

// import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.Supplier;

// import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
// import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
// import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.config.PIDConstants;

import frc.robot.autonomous.MoveToPose;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.Constants.Drivetrain;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    private Optional<Alliance> alliance;
    // public final MoveToPose moveToPose;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private final SwerveRequest.ApplyRobotSpeeds m_robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private static final PathConstraints pathConstraints =
    new PathConstraints(Drivetrain.kMaxLateralSpeed, Drivetrain.kMaxLateralAcceleration,
        Drivetrain.kMaxAngularSpeed, Drivetrain.kMaxAngularAcceleration);

    public double getPoseDifference(final Pose2d input_pose) {
        return this.getState().Pose.getTranslation().getDistance(input_pose.getTranslation());
    }

    public Pose2d getPose2d(){
        return this.getState().Pose;
    }
    
    public ChassisSpeeds getChassisSpeeds() {
        return this.getKinematics().toChassisSpeeds(getState().ModuleStates);
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        m_robotSpeeds.Speeds = speeds;  // Set the desired robot-relative speeds
        this.setControl(m_robotSpeeds); // Apply the control request to the drivetrain
    }
        

    // /* The SysId routine to test */
    // private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    // public CommandSwerveDrivetrain(
    //     SwerveDrivetrainConstants drivetrainConstants,
    //     SwerveModuleConstants<?, ?, ?>... modules
    // ) {
    //     super(drivetrainConstants, modules);
    //     if (Utils.isSimulation()) {
    //         startSimThread();
    //     }
    // }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        RobotConfig config;
        

        try{
            config = RobotConfig.fromGUISettings();
      
          // Configure AutoBuilder last
          AutoBuilder.configure(
                  this::getPose2d, // Robot pose supplier
                  this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                  this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                  (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                            new PIDConstants(Drivetrain.kLateralPositionP, 0.0, Drivetrain.kLateralPositionD), // Translation PID constants
                            new PIDConstants(Drivetrain.kAngularPositionP, 0.0, Drivetrain.kAngularPositionD) // Rotation PID constants
                  ),
                  config, // The robot configuration
                  () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.

                    // ORIGIN WILL ALWAYS BE BLUE - Jadyn
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      
                    alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                      return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                  },
                  this // Reference to this subsystem to set requirements
          );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
          }

    }

public Command AutoAlign(final List<Pose2d> targetPoses) {
    Pose2d targetPose = getPose2d().nearest(targetPoses);

    if (DriverStation.getAlliance().equals(Alliance.Blue)) {
        return AutoBuilder.pathfindToPose(targetPose, pathConstraints);
    } else {
        return AutoBuilder.pathfindToPoseFlipped(targetPose, pathConstraints);
    }
} 


public Command findAndFollowPath(final Pose2d targetPose) {

    if (DriverStation.getAlliance().equals(Alliance.Blue)) {
        return AutoBuilder.pathfindToPose(targetPose, pathConstraints);
    } else {
        return AutoBuilder.pathfindToPoseFlipped(targetPose, pathConstraints);
    }
} 



// public Command AutoAlign(final Pose2d targetPose, final double finaltimeout){
//     return new SequentialCommandGroup(
//         // Path Gen and Follower
//         new InstantCommand(() -> {
//             if (DriverStation.getAlliance().equals(Alliance.Blue)) {
//                 System.out.println("DEBUG: RUNNING PATHFINDER BLUE");
//                 AutoBuilder.pathfindToPose(targetPose, pathConstraints);
//             } else {
//                 AutoBuilder.pathfindToPoseFlipped(targetPose, pathConstraints);
//                 System.out.println("DEBUG: RUNNING PATHFINDER RED");
//             }
//         }),

//         new InstantCommand(() -> {
//             new MoveToPose(targetPose, this).withTimeout(finaltimeout);
//             System.out.println("DEBUG: RUNNING MOVE TO POSE");
//         })
        
//         // End of Sequential CommandGroup
//     );
// }

    



  public Command followPath(final PathPlannerPath path, boolean fromfile) {

    if (DriverStation.getAlliance().isPresent() == false){
        return new Command() {};
    }

    if (DriverStation.getAlliance().equals(Alliance.Blue)){
      return AutoBuilder.followPath(path);
    }
    else{
      return AutoBuilder.followPath(path.flipPath());
    }
  }


    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    // public CommandSwerveDrivetrain(
    //     SwerveDrivetrainConstants drivetrainConstants,
    //     double odometryUpdateFrequency,
    //     Matrix<N3, N1> odometryStandardDeviation,
    //     Matrix<N3, N1> visionStandardDeviation,
    //     SwerveModuleConstants<?, ?, ?>... modules
    // ) {
    //     super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
    //     if (Utils.isSimulation()) {
    //         startSimThread();
    //     }

    //     RobotConfig config;
        

    //     try{
    //         config = RobotConfig.fromGUISettings();
    //       } catch (Exception e) {
    //         // Handle exception as needed
    //         e.printStackTrace();
    //       }
      
    //       // Configure AutoBuilder last
    //       AutoBuilder.configure(
    //               this::getPose2d, // Robot pose supplier
    //               this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
    //               this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    //               (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
    //               new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
    //                       new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    //                       new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
    //               ),
    //               config, // The robot configuration
    //               () -> {
    //                 // Boolean supplier that controls when the path will be mirrored for the red alliance
    //                 // This will flip the path being followed to the red side of the field.

    //                 // ORIGIN WILL ALWAYS BE BLUE - Jadyn
    //                 // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      
    //                 var alliance = DriverStation.getAlliance();
    //                 if (alliance.isPresent()) {
    //                   return alliance.get() == DriverStation.Alliance.Red;
    //                 }
    //                 return false;
    //               },
    //               this // Reference to this subsystem to set requirements
    //       );
    //     }
      
    

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    // // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // //     return m_sysIdRoutineToApply.quasistatic(direction);
    // // }

    // /**
    //  * Runs the SysId Dynamic test in the given direction for the routine
    //  * specified by {@link #m_sysIdRoutineToApply}.
    //  *
    //  * @param direction Direction of the SysId Dynamic test
    //  * @return Command to run
    //  */
    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return m_sysIdRoutineToApply.dynamic(direction);
    // }
    

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}

