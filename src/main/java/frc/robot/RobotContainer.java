// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
// import java.io.Console;
// import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
// import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.PhotonUtils;
// import org.photonvision.targeting.PhotonTrackedTarget;


import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.PhotonVisionHandler;
// import frc.robot.Vision.MeasurementInfo;


public class RobotContainer {

        // 6 meters per second desired top speed.
    private static final double kMaxSpeed = 0.2;
    PowerDistribution m_powerdistro = new PowerDistribution();

    private Optional<EstimatedRobotPose> prevVisionOut = Optional.empty();
    private Optional<EstimatedRobotPose> Visionout;
    private final Field2d m_Visionpose = new Field2d();

    public final PhotonVisionHandler visionHandler = new PhotonVisionHandler();
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    // Vision visionInstance;

    // Half a rotation per second max angular velocity.
    private static final double kMaxAngularRate = 0.2 * Math.PI;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    private final SwerveRequest.FieldCentricFacingAngle m_angleRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(kMaxSpeed * 0.05)
    .withRotationalDeadband(kMaxAngularRate * 0.05)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
    .withDeadband(kMaxSpeed * 0.05).withRotationalDeadband(kMaxAngularRate * 0.05) // 20% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // closed loop velocity control


    // Velocity not tuned ????

    /* Setting up bindings for necessary control of the swerve drive platform */

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandXboxController m_controller = new CommandXboxController(0);

    public final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    private double getLeftY() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
        return m_controller.getLeftY();
        return -m_controller.getLeftY();
    }

    private double getLeftX() {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red)
        return m_controller.getLeftX();
        return -m_controller.getLeftX();
    }


    public RobotContainer() {
        configureBindings();
        m_powerdistro.setSwitchableChannel(true);
    }
    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(() -> m_drive.withVelocityX(-m_controller.getLeftY() * kMaxSpeed)
            .withVelocityY(-m_controller.getLeftX() * kMaxSpeed)
            .withRotationalRate(-m_controller.getRightX() * kMaxAngularRate)));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        m_controller.leftBumper().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

        m_drivetrain.registerTelemetry(logger::telemeterize);
    }



    // Pose estimator update logic (meant to increase accuracy by filtering out bad or unusable output from the Cameras)
    public void updatePoseEstimator() {

    if (aprilTagFieldLayout == null) {
      System.err.println("AprilTagFieldLayout is null. Skipping pose estimation update.");
      return;
    }

    double lateralDeviation; // standard deviation of the x and y measurements
    double angularDeviation; // standard deviation of the angle measurement
    // final MeasurementInfo internalTag =
    // visionInstance.new MeasurementInfo(visionHandler.getAprilTagID(),
    // visionHandler.getNumberofTags(), visionHandler.areaOfAprilTag());


    //Feedback logic for Photonvision Pose estimator (Kinda jank but ok for now)
    if (prevVisionOut.isPresent()) {
      System.out.println("YES Visionout");
      Visionout = visionHandler.getEstimatedGlobalPose(prevVisionOut.get().estimatedPose.toPose2d());
    } else {
      System.out.println("NO Visionout");
      Visionout = visionHandler.getEstimatedGlobalPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    }

    prevVisionOut = Visionout;


    try {
      
//Make Get Methods For Tag Size, # of Tags, 
//Pos dif function for gives dif of given pos and robot 


          // TODO: Redo the Fusion Logic to work with the Photonvision Cams. 
          // NOTE: (may not even need it, should get a deeper understanding of the photonvision Pose estimator)

          if (Visionout.isPresent()) {

          final Pose2d visPose = Visionout.get().estimatedPose.toPose2d();
          final double posDiff = m_drivetrain.getPoseDifference(visPose);
    
          final List<PhotonTrackedTarget> tags = Visionout.get().targetsUsed;

          //Set and Put Output from Vision on Smart Dashboard for debugging
          m_Visionpose.setRobotPose(Visionout.get().estimatedPose.toPose2d());
          SmartDashboard.putData("Vision Pose", m_Visionpose);
          
          // // return if no tag detected
          if (tags.size() < 1) {
            return;
          }
          // // more than 1 tag in view
          if (tags.size() > 1 && visionHandler.avgTagArea(tags) > 80) {
            lateralDeviation = 0.5;
            angularDeviation = 6;
          }
          // // 1 target with large area and close to estimated pose
          else if (tags.get(0).getArea() > 80 && posDiff < 0.5) {
            lateralDeviation = 1.0;
            angularDeviation = 12;
          }
          // 1 target farther away and estimated pose is close
          else if (tags.get(0).getArea() > 10 && posDiff < 0.3) {
            lateralDeviation = 2.0;
            angularDeviation = 30;
          }
          // conditions don't match to add a vision measurement
          else{
            return;
          }    
              
        // Only fuse with WPIlib Kalman filter (Basically our Robotpose) when sim is off to prevent jank
          if (Utils.isSimulation() == false)
    
          {
    
            Pose2d visPose2d = Visionout.get().estimatedPose.toPose2d();
            double visionstamp = Visionout.get().timestampSeconds;
            m_drivetrain.addVisionMeasurement(visPose2d, visionstamp, VecBuilder.fill(lateralDeviation,
                lateralDeviation, Units.degreesToRadians(angularDeviation)));
          }
        }
        //}
  
    } catch (Exception e) {

      System.out.println(e);
      // TODO: handle exception
    }
 }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
