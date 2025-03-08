// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.networktables.BooleanPublisher;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.PhotonUtils;
// import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.ElevatorEncoder;
import frc.robot.subsystems.ElevatorMM;
import frc.robot.subsystems.Intake;
import frc.robot.util.PhotonVisionHandler;
// import frc.robot.Vision.MeasurementInfo;
// import frc.robot.util.koralSensorWrapper;


public class RobotContainer {

    // 6 meters per second desired top speed.
    PowerDistribution m_powerdistro = new PowerDistribution();
    private boolean toggleState = false; // Track state


    private Optional<EstimatedRobotPose> prevVisionOutFront = Optional.empty();
    private Optional<EstimatedRobotPose> prevVisionOutBack = Optional.empty();

    private Optional<EstimatedRobotPose> VisionoutFront;
    private Optional<EstimatedRobotPose> VisionoutBack;

    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<RobotState> armstates = new SendableChooser<>();

    private final Field2d m_VisionposeFront = new Field2d();
    private final Field2d m_VisionposeBack = new Field2d();

    private final Field2d m_Fieldpose = new Field2d();

    public final Intake m_Intake = new Intake();
    public static final Arm m_Arm = new Arm(); 
    public final ElevatorMM m_Elevator = new ElevatorMM();
    public static final DigitalInput m_koral_sensor = new DigitalInput(0);
    Trigger objectDetected = new Trigger(m_koral_sensor::get);

    Trigger robotMoving = new Trigger(() -> {

      if(Math.abs(m_controller.getLeftY()) > 0 || Math.abs(m_controller.getLeftY()) > 0 || Math.abs(m_controller.getRightX()) > 0 || Math.abs(m_controller.getRightY()) > 0){
        return true;
      } else{
        return false;
      }
      
    });

    public enum RobotState {
      TOP,
      MID,
      LOW,
      BOTTOM;
  }

    private final Transform3d robotToCamBack =
      new Transform3d(new Translation3d(Units.inchesToMeters(-0.625), Units.inchesToMeters(0),
          Units.inchesToMeters(35.10)), new Rotation3d(0, Math.toRadians(15), 0)); // Adjusted


    // TODO: GET ROBOT TO CAM OFFSETS FROM CARA TMRW FOR FRONT CAM
    private final Transform3d robotToCamFront =
          new Transform3d(new Translation3d(Units.inchesToMeters(-0.625), Units.inchesToMeters(0),
              Units.inchesToMeters(35.10)), new Rotation3d(0, Math.toRadians(15), 0)); // Adjusted
    
    
    public final PhotonVisionHandler visionHandlerBack = new PhotonVisionHandler("Back", robotToCamBack);
    public final PhotonVisionHandler visionHandlerFront = new PhotonVisionHandler("Front", robotToCamFront);

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    // Vision visionInstance;

    // Half a rotation per second max angular velocity.
    private static final double kMaxAngularRate = 1.00 * Math.PI;
    private static final double kMaxSpeed = 1.00;

    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    private final SwerveRequest.FieldCentricFacingAngle m_angleRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(kMaxSpeed * 0.1)
    .withRotationalDeadband(kMaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
    .withDeadband(kMaxSpeed * 0.1).withRotationalDeadband(kMaxAngularRate * 0.1) // 20% deadband
    .withDriveRequestType(DriveRequestType.Velocity); // closed loop velocity control


    // Velocity not tuned ????

    /* Setting up bindings for necessary control of the swerve drive platform */

    private final Telemetry logger = new Telemetry(kMaxSpeed);
    private double loadangle = 0.26;

    public static final CommandXboxController m_controller = new CommandXboxController(0);

    public final static CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

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

    // private Command testing(boolean tof){
    //   return new InstantCommand( () ->{

    //     if(tof){
    //       System.out.println("TRUE");
    //       System.out.println("TRUE");
    //       System.out.println("TRUE");
    //       System.out.println("TRUE");

    //     } else{
    //       System.out.println("FALSE");
    //       System.out.println("FALSE");
    //       System.out.println("FALSE");
    //       System.out.println("FALSE");
    //     }

    //   });
    // }


    public RobotContainer() {
        configureBindings();

      
        m_powerdistro.setSwitchableChannel(true);

                // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Koral Trigger", m_koral_sensor);
        SmartDashboard.putData("Elevator", m_Elevator);

        SmartDashboard.putData("Arm", m_Arm);
        SmartDashboard.putData("intake", m_Intake);

        armstates.setDefaultOption("Top", RobotState.TOP);
        armstates.addOption("Mid", RobotState.MID);
        armstates.addOption("Low", RobotState.LOW);
        armstates.addOption("Bottom", RobotState.BOTTOM);
    }

    private Command l1Command(){
      return new SequentialCommandGroup(
        m_Elevator.goToHeight(0.75),
        m_Arm.goToAngle(0.26 * 0.70).withTimeout(1.0),
        new WaitCommand(1),
        new InstantCommand(() -> loadangle = 0.26 * 0.70)
      );
    }

    private Command l2Command(){
      return new SequentialCommandGroup(
        m_Elevator.goToHeight(1.17),
        new WaitCommand(0.5),
        m_Arm.goToAngle(0.26 * 0.65).withTimeout(1.0),
        new WaitCommand(1),
        new InstantCommand(() -> loadangle = 0.26 * 0.65)
      );
    }

    private Command l3Command(){
      return new SequentialCommandGroup(
        m_Elevator.goToHeight(2.62),
        new WaitCommand(0.5),
        m_Arm.goToAngle(0.26 * 0.65).withTimeout(1.0),
        new WaitCommand(1),
        new InstantCommand(() -> loadangle = 0.26 * 0.65)
      );
    }

    private Command algaeclearTop(){
      return new SequentialCommandGroup(
        m_Elevator.goToHeight(2.40),
        new WaitCommand(0.5),
        m_Arm.goToAngle(0.26 * 0.50).withTimeout(1.0),
        new WaitCommand(0.5),
        new InstantCommand(() -> loadangle = 0.26 * 0.65),
        new WaitCommand(0.5),
        m_Intake.stop()
      );
    }

    private Command algaeclearBottom(){
      return new SequentialCommandGroup(
        m_Elevator.goToHeight(0.90),
        new WaitCommand(0.5),
        m_Arm.goToAngle(0.26 * 0.55).withTimeout(1.0),
        new WaitCommand(0.5),
        new InstantCommand(() -> loadangle = 0.26 * 0.60),
        new WaitCommand(0.5),
        m_Intake.stop()

      );
    }

    private Command l4Command(){
      return new SequentialCommandGroup(
        m_Arm.goToAngle(0.26 * 0.67).withTimeout(1.5),
        m_Elevator.goToHeight(4.94),
        new WaitCommand(1),
        new InstantCommand(() -> loadangle = 0.26 * 0.67),
        // new WaitCommand(1),
        // m_Intake.reverse(5),
        // new WaitCommand(0.01),
        m_Intake.stop()
      );
    }

    private Command resetElevatorCmd(){
      return new ParallelCommandGroup(
        m_Elevator.goToHeight(0.05),
        m_Intake.forwards(true),
        m_Arm.goToAngle(0.26)
      );
    }

    private Command algaeOutCmd(){
      return new SequentialCommandGroup(
        m_Elevator.goToHeight(0.05),
        new WaitCommand(1),
        m_Arm.goToAngle(0),
        new WaitCommand(0.25),
        m_Intake.forwardsame()
      );
    }

    //For scoring Algue Under Author Aaron 
    private Command algueScoreCmd(){
      return new SequentialCommandGroup(
        m_Intake.forwardsame(),
        m_Elevator.goToHeight(1.1), //TODO get from Shuffle board
        m_Arm.goToAngle(-0.05)
        // m_Arm.goToAngle(0), 
      );
    }

    private Command outTakeCmd(){
      return new SequentialCommandGroup(
        m_Arm.goToAngle(loadangle),
        m_Intake.forwards(false).withTimeout(1)
      );
    }
    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(() -> m_drive.withVelocityX(m_controller.getLeftY() * kMaxSpeed)
            .withVelocityY(m_controller.getLeftX() * kMaxSpeed)
            .withRotationalRate(-m_controller.getRightX() * kMaxAngularRate))
          );

        NamedCommands.registerCommand("L1", l1Command());
        NamedCommands.registerCommand("L2", l2Command());
        NamedCommands.registerCommand("L3", l3Command());
        NamedCommands.registerCommand("L4", l4Command());
        NamedCommands.registerCommand("Score Koral", outTakeCmd());
        NamedCommands.registerCommand("Reset Elevator", resetElevatorCmd());
      

        // reset the field-centric heading on left bumper press
        // m_controller.b().onTrue(m_drivetrain.findAndFollowPath(new Pose2d(14.7, 4.045, new Rotation2d(Units.degreesToRadians(180)))));
        // m_controller.y().onTrue(m_drivetrain.findAndFollowPath(new Pose2d(15, 5.063, new Rotation2d(Units.degreesToRadians(180)))));

        // Inside configureBindings()
        // Intake should run forwards while an object is detected

        // m_controller.a().toggleOnTrue(new InstantCommand(() -> {
        //   toggleState = !toggleState; // Flip the toggle
        //   m_Arm.goToAngle(toggleState ? 0.29 : 0.0).schedule(); // Choose angle
        // }))


        // robotMoving.onTrue(m_Arm.goToAngle(0.28));

        // m_controller.rightTrigger().onTrue(m_Arm.goToAngle(0.29));
//.andThen(m_Intake.reverse())


        // L1
        m_controller.a().onTrue(l1Command());

        m_controller.b().onTrue(l2Command());

        m_controller.y().onTrue(l3Command());

        m_controller.x().onTrue(l4Command());

        m_controller.rightBumper().onTrue(algaeclearTop());

        m_controller.leftBumper().onTrue(algaeclearBottom());
        
        m_controller.povRight().onTrue(algueScoreCmd()); 
        
        // m_controller.povLeft().onTrue(algaeOutCmd());
        m_controller.povLeft().onTrue(m_Intake.reversesame());

        m_controller.povDown().onTrue(m_Intake.stop());


        // m_controller.a().onTrue(m_Elevator.goToHeight(2));
        // m_controller.y().onTrue(m_Elevator.goToHeight(1));
        m_controller.rightTrigger().onTrue(m_Elevator.goToHeight(0.05).alongWith(m_Arm.goToAngle(0.26)).alongWith(m_Intake.forwards(true)));

        m_controller.leftTrigger()
        .onTrue(m_Arm.goToAngle(loadangle).andThen(m_Intake.forwards(false).withTimeout(1)))
        .onFalse(resetElevatorCmd());

        // m_controller.povDown().onTrue(m_Intake.forwards(true));

        objectDetected.onFalse(m_Intake.koralControlCommand(0.075)); //-0.38
        objectDetected.onTrue(m_Intake.forwards(true));
        

        // objectDetected.and().whileTrue(m_Arm.goToAngle(0));
        // objectDetected
        //   .onFalse(testing(false))
        //   .onTrue(testing(true))
        // ;

        // Manual override using controller buttons 
        // m_controller.rightBumper().onTrue(m_Intake.forwards());
        // m_controller.leftBumper().onTrue(m_Intake.reverse());
        // m_controller.leftBumper().onFalse(m_Intake.forwards());
      
        // m_controller.x().onTrue(m_Intake.stop());
        
        m_controller.povUp().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

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
    if (prevVisionOutFront.isPresent()) {
      // System.out.println("YES Visionout");
      VisionoutFront = visionHandlerFront.getEstimatedGlobalPose(prevVisionOutFront.get().estimatedPose.toPose2d());
    } else {
      // System.out.println("NO Visionout");
      VisionoutFront = visionHandlerFront.getEstimatedGlobalPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    }

    if (prevVisionOutBack.isPresent()) {
      // System.out.println("YES Visionout");
      VisionoutBack = visionHandlerBack.getEstimatedGlobalPose(prevVisionOutBack.get().estimatedPose.toPose2d());
    } else {
      // System.out.println("NO Visionout");
      VisionoutBack = visionHandlerBack.getEstimatedGlobalPose(new Pose2d(new Translation2d(0, 0), new Rotation2d(0)));
    }


    prevVisionOutBack = VisionoutBack;
    prevVisionOutFront = VisionoutFront;

    try {
      
//Make Get Methods For Tag Size, # of Tags, 
//Pos dif function for gives dif of given pos and robot 


        // TODO: Redo the Fusion Logic to work with the Photonvision Cams. 
        // NOTE: (may not even need it, should get a deeper understanding of the photonvision Pose estimator)

        if (VisionoutBack.isPresent()) {

        final Pose2d visPose = VisionoutBack.get().estimatedPose.toPose2d();
        final double posDiff = m_drivetrain.getPoseDifference(visPose);

        final List<PhotonTrackedTarget> tags = VisionoutBack.get().targetsUsed;

        // System.out.println("posDiff " + posDiff);
        // System.out.println("Tag Area" + visionHandler.avgTagArea(tags));
        
        //Set and Put Output from Vision on Smart Dashboard for debugging
        m_VisionposeBack.setRobotPose(VisionoutBack.get().estimatedPose.toPose2d());
        SmartDashboard.putData("Vision Pose Back", m_VisionposeBack);
        
        // // return if no tag detected
        if (tags.size() < 1) {
          // System.out.println("No tags to fuse");
          return;
        } 
        // // more than 1 tag in view
        if (tags.size() > 1 && visionHandlerBack.avgTagArea(tags) > 0.8) {
          // System.out.println("Fuse state 1");

          lateralDeviation = 0.5;
          angularDeviation = 6;
        }
        // // 1 target with large area and close to estimated pose
        //  && posDiff < 0.5
        else if (tags.get(0).getArea() > 0.8) {
          // System.out.println("Fuse state 2");

          lateralDeviation = 1.0;
          angularDeviation = 12;
        }
        // 1 target farther away and estimated pose is close
        //  && posDiff < 0.3
        else if (tags.get(0).getArea() > 0.1) {
          // System.out.println("Fuse state 3");

          lateralDeviation = 2.0;
          angularDeviation = 30;
        }
        // conditions don't match to add a vision measurement
        else{
          // System.out.println("Unable to fuse (Conditions not Met)");

          return;
        }    
            
      // Only fuse with WPIlib Kalman filter (Basically our Robotpose) when the sim is off to prevent janky movement
        if (!Utils.isSimulation())
  
        {

          // System.out.println("Fusion Successful");
  
          Pose2d visPose2d = VisionoutBack.get().estimatedPose.toPose2d();

          // Pose2d testpose = new Pose2d(visPose2d.getTranslation(), m_drivetrain.getRotation3d().toRotation2d());
          double visionstamp = VisionoutBack.get().timestampSeconds;

          m_drivetrain.addVisionMeasurement(visPose2d, visionstamp, VecBuilder.fill(lateralDeviation,
              lateralDeviation, Units.degreesToRadians(angularDeviation)));


        }
      }

      m_Fieldpose.setRobotPose(m_drivetrain.getState().Pose);
      SmartDashboard.putData("Robot Pose", m_Fieldpose);
      //}
  
    } catch (Exception e) {

      System.out.println(e);
      // TODO: handle exception
    }

  try {
    
    //Make Get Methods For Tag Size, # of Tags, 
    //Pos dif function for gives dif of given pos and robot 
    
    
    // TODO: Redo the Fusion Logic to work with the Photonvision Cams. 
    // NOTE: (may not even need it, should get a deeper understanding of the photonvision Pose estimator)

    if (VisionoutFront.isPresent()) {

    final Pose2d visPoseFront = VisionoutFront.get().estimatedPose.toPose2d();

    double lateralDeviationFront;
    double angularDeviationFront;
    // final double posDiff = m_drivetrain.getPoseDifference(visPose);

    final List<PhotonTrackedTarget> tags = VisionoutFront.get().targetsUsed;

    // System.out.println("posDiff " + posDiff);
    // System.out.println("Tag Area" + visionHandler.avgTagArea(tags));
    
    //Set and Put Output from Vision on Smart Dashboard for debugging
    m_VisionposeFront.setRobotPose(VisionoutFront.get().estimatedPose.toPose2d());
    SmartDashboard.putData("Vision Pose Front", m_VisionposeFront);
    
    // // return if no tag detected
    if (tags.size() < 1) {
      // System.out.println("No tags to fuse");
      return;
    } 
    // // more than 1 tag in view
    if (tags.size() > 1 && visionHandlerFront.avgTagArea(tags) > 0.8) {
      // System.out.println("Fuse state 1");

      lateralDeviationFront = 0.5;
      angularDeviationFront = 6;
    }
    // // 1 target with large area and close to estimated pose
    //  && posDiff < 0.5
    else if (tags.get(0).getArea() > 0.8) {
      // System.out.println("Fuse state 2");

      lateralDeviationFront = 1.0;
      angularDeviationFront = 12;
    }
    // 1 target farther away and estimated pose is close
    //  && posDiff < 0.3
    else if (tags.get(0).getArea() > 0.1) {
      // System.out.println("Fuse state 3");

      lateralDeviationFront = 2.0;
      angularDeviationFront = 30;
    }
    // conditions don't match to add a vision measurement
    else{
      // System.out.println("Unable to fuse (Conditions not Met)");

      return;
    }    
        
  // Only fuse with WPIlib Kalman filter (Basically our Robotpose) when the sim is off to prevent janky movement
    if (!Utils.isSimulation())

    {

      // System.out.println("Fusion Successful");

      Pose2d visPose2dFront = VisionoutFront.get().estimatedPose.toPose2d();

      // Pose2d testpose = new Pose2d(visPose2d.getTranslation(), m_drivetrain.getRotation3d().toRotation2d());
      double visionstampFront = VisionoutFront.get().timestampSeconds;

      m_drivetrain.addVisionMeasurement(visPose2dFront, visionstampFront, VecBuilder.fill(lateralDeviationFront,
      lateralDeviationFront, Units.degreesToRadians(angularDeviationFront)));
    }
  }

      
  } catch (Exception e) {

      System.out.println(e);
      // TODO: handle exception
    }
      
 }


    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }
}
