// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.text.NumberFormat.Style;
import java.util.List;
// import java.io.Console;
// import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.jar.Attributes.Name;

import org.photonvision.EstimatedRobotPose;
// import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
// import edu.wpi.first.networktables.BooleanPublisher;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.autonomous.MoveToPose;
// import frc.robot.autonomous.ReefAlign;
// import frc.robot.autonomous.ReefAlignMovement;

// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.PhotonPoseEstimator.PoseStrategy;
// import org.photonvision.PhotonUtils;
// import org.photonvision.targeting.PhotonTrackedTarget;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.ElevatorEncoder;
import frc.robot.subsystems.ElevatorMM;
import frc.robot.subsystems.Intake;
import frc.robot.util.Constants;
import frc.robot.util.PhotonVisionHandler;
// import frc.robot.Vision.MeasurementInfo;
// import frc.robot.util.koralSensorWrapper;
import frc.robot.util.PoseEstimatorInst;


public class RobotContainer {
  // private boolean toggleStateRamp = false; // Boolean to store the state
  // private BooleanEntry toggleStateEntry;  // Entry to display the boolean on Shuffleboard
  // private ShuffleboardTab shuffleboardTab;  // Shuffleboard Tab

  // tab.add("Ramp Release Enable", false).getEntry();

    // 6 meters per second desired top speed.
    PowerDistribution m_powerdistro = new PowerDistribution();
    private boolean toggleState = false; // Track state

    private PoseEstimatorInst rightPoseEstimator;
    private PoseEstimatorInst leftPoseEstimator;

    public Climb m_climber = new Climb();

    private boolean climbrunce = false;

    private Optional<EstimatedRobotPose> prevVisionOutFront = Optional.empty();
    private Optional<EstimatedRobotPose> prevVisionOutBack = Optional.empty();

    private Optional<EstimatedRobotPose> VisionoutFront;
    private Optional<EstimatedRobotPose> VisionoutBack;

    private final SendableChooser<Command> autoChooser;

    private final Field2d m_VisionposeFront = new Field2d();
    private final Field2d m_VisionposeBack = new Field2d();

    private final Field2d m_Fieldpose = new Field2d();

    public final Intake m_Intake = new Intake();
    public static final Arm m_Arm = new Arm(); 
    public final ElevatorMM m_Elevator = new ElevatorMM();
    public static final DigitalInput m_koral_sensor = new DigitalInput(0);
    public static final Servo m_rampRelease1 = new Servo(1); 
    public static final Servo m_rampRelease2 = new Servo(2); 

    Trigger objectDetected = new Trigger(() -> m_koral_sensor.get());

    // Trigger robotMoving = new Trigger(() -> {

    //   if(Math.abs(m_controller.getLeftY()) > 0.1 || Math.abs(m_controller.getLeftY()) > 0.1 || Math.abs(m_controller.getRightX()) > 0.1 || Math.abs(m_controller.getRightY()) > 0.1){
    //     return true;
    //   } else{
    //     return false;
    //   }
      
    // });

    //35.10

    // x = 8 inches

    private final Transform3d robotToCamLeft =
      new Transform3d(new Translation3d(Units.inchesToMeters(8), Units.inchesToMeters(13),
          Units.inchesToMeters(13.50)), new Rotation3d(0, 0, Math.toRadians(-30))); // Adjusted


    // TODO: GET ROBOT TO CAM OFFSETS FROM CARA TMRW FOR FRONT CAM
    private final Transform3d robotToCamRight =
          new Transform3d(new Translation3d(Units.inchesToMeters(8), Units.inchesToMeters(-13),
              Units.inchesToMeters(13.50)), new Rotation3d(0, 0, Math.toRadians(30))); // Adjusted
    
    
    public final PhotonVisionHandler visionHandlerLeft = new PhotonVisionHandler("Back", robotToCamLeft);
    public final PhotonVisionHandler visionHandlerRight = new PhotonVisionHandler("Front", robotToCamRight);

    // public final ReefAlign m_ReefAlign = new ReefAlign(m_drivetrain, visionHandlerFront);

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    // Vision visionInstance;

    // Half a rotation per second max angular velocity.
    private static final double kMaxAngularRate = 4.0 * Math.PI;
    private static final double kMaxSpeed = 4.0;

    private static final double kAngulardeadband = kMaxAngularRate * 0.1;
    private static final double kLineardeadband = kMaxSpeed * 0.1;


    // private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    // private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    private final SwerveRequest.FieldCentricFacingAngle m_angleRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(kLineardeadband)
    .withRotationalDeadband(kAngulardeadband)
    .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
    .withDeadband(kLineardeadband)
    .withRotationalDeadband(kAngulardeadband) // 20% deadband
    .withDriveRequestType(DriveRequestType.Velocity); // closed loop velocity control


    // Velocity not tuned ????

    /* Setting up bindings for necessary control of the swerve drive platform */

    // private final Telemetry logger = new Telemetry(kMaxSpeed);
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
 
    final private double expoCurve(double input, final double a, final double deadband){
      double absinput = Math.abs(input);
      final double inverseA = (1.0 / a);
      
      if(absinput < deadband){
        return 0;
      }
      return ((Math.pow(a, absinput) * input * inverseA) + (deadband * Math.signum(input)));
        
    }

  //   private void updateShuffleboard() {
  //     // This method will update the display on Shuffleboard based on the current boolean state
  //     ((GenericPublisher) toggleStateEntry).setBoolean(toggleStateRamp);
  // }

    
    public RobotContainer() {
        configureBindings();

        m_rampRelease1.set(0);
        m_rampRelease2.set(0);


        rightPoseEstimator = new PoseEstimatorInst(visionHandlerRight, m_drivetrain, m_VisionposeFront);
        leftPoseEstimator = new PoseEstimatorInst(visionHandlerLeft, m_drivetrain, m_VisionposeBack);
  
        // m_powerdistro.setSwitchableChannel(true);
        // if (m_powerdistro.getStickyFaults()) {
          
          
        // }
        System.out.print(m_powerdistro.getStickyFaults()); 
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Elevator", m_Elevator);

        // SmartDashboard.putData("Arm", m_Arm);
        // SmartDashboard.putData("intake", m_Intake);

        // SmartDashboard.putData("climb", m_climber);

        // shuffleboardTab = Shuffleboard.getTab("Driver");  // Create or get the "Driver" tab

        // // Add a button to Shuffleboard that will toggle the boolean
        // shuffleboardTab.add("Toggle Ramp Release", new InstantCommand(() -> toggleStateRamp = !toggleStateRamp));

        // // Add a BooleanEntry to display the current state of toggleStateRamp on Shuffleboard
        // toggleStateEntry = (BooleanEntry) shuffleboardTab.add("Ramp Release Enabled", toggleStateRamp).getEntry();

        // // Update the boolean entry whenever the toggleStateRamp changes
        // updateShuffleboard();


    }

    private Command zeroheight(){
      return new SequentialCommandGroup(
        m_Elevator.goToHeight(0.05)

      );
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
        new WaitCommand(0.5)
      );
    }

    private Command algaeclearBottom(){
      return new SequentialCommandGroup(
        m_Elevator.goToHeight(0.90),
        new WaitCommand(0.5),
        m_Arm.goToAngle(0.26 * 0.55).withTimeout(1.0),
        new WaitCommand(0.5),
        new InstantCommand(() -> loadangle = 0.26 * 0.60),
        new WaitCommand(0.5)
      );
    }

    private Command l4Command(){
      return new SequentialCommandGroup(
        m_Arm.goToAngle(0.26 * 0.67).withTimeout(1.5),
        m_Elevator.goToHeight(4.955),
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
        m_Arm.goToAngle(0.265)
      );
    }

    private Command algaeOutCmd(){
      return new SequentialCommandGroup(
        m_Arm.goToAngle(0),
        new WaitCommand(0.25),
        m_Intake.reversesame(),
        new WaitCommand(1)
        // m_Elevator.goToHeight(0.05)

      );
    }

    //For scoring Algue Under 
    private Command algueScoreCmd(){
      return new SequentialCommandGroup(
        m_Intake.forwardsame(),
        m_Elevator.goToHeight(1.1),
        m_Arm.goToAngle(-0.05)
        // m_Arm.goToAngle(0), 
      );
    }

    private Command outTakeCmd(){
      return new SequentialCommandGroup(
        m_Arm.goToAngle(loadangle),
        m_Intake.forwards(false).withTimeout(1),
        m_Intake.forwards(true)
      );
    }


    private Command waituntilKoral(){
      return new SequentialCommandGroup(
      Commands.waitUntil(() -> !  objectDetected.getAsBoolean()),
      new WaitCommand(0.075),
      m_Intake.stop()
      );
    }


    private Command ClimberControlLogic(){
      return new InstantCommand(() ->{

          // if(!climbrunce){
          //   m_climber.goToPosition(75);
          //   climbrunce = true;
          // } else{
            // m_climber.setVelocity(120);

            m_climber.setClimbVoltage(10);
          //}

      });
    }

    private Command ClimberGoUp(){
      return new InstantCommand(() ->{
            m_climber.goToPosition(90).schedule();
      });
    }

    private Command RampRelease(double val){
      return new InstantCommand(() ->{
        m_rampRelease1.set(val);
        m_rampRelease2.set(val);  
        
      });
    }

    
    
    private void configureBindings() {
        RampRelease(1);
        // m_rampRelease1.set(1);
        // m_rampRelease2.set(1);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(() -> m_drive.withVelocityX(-expoCurve(m_controller.getLeftY(), 20, 0.1) * kMaxSpeed)
            .withVelocityY(-expoCurve(m_controller.getLeftX(), 20, 0.1) * kMaxSpeed)
            .withRotationalRate(expoCurve(-m_controller.getRightX(), 20, 0.1) * kMaxAngularRate))
        );

        // m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        // m_drivetrain.applyRequest(() -> m_drive.withVelocityX(m_controller.getLeftY() * kMaxSpeed)
        //     .withVelocityY(m_controller.getLeftX() * kMaxSpeed)
        //     .withRotationalRate(-m_controller.getRightX() * kMaxAngularRate))
        // );

        // m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        //   RampRelease(m_controller.getLeftY())
        // );


        // NamedCommands.registerCommand(, getAutonomousCommand());
        NamedCommands.registerCommand("Wait For Coral", waituntilKoral());
        NamedCommands.registerCommand("zeroHeight", zeroheight());
        NamedCommands.registerCommand("L1", l1Command());
        NamedCommands.registerCommand("L2", l2Command());
        NamedCommands.registerCommand("L3", l3Command());
        NamedCommands.registerCommand("L4", l4Command());
        NamedCommands.registerCommand("Score Koral", outTakeCmd());
        NamedCommands.registerCommand("Reset Elevator", resetElevatorCmd());
        NamedCommands.registerCommand("ArmUp", m_Arm.goToAngle(0.26));
        NamedCommands.registerCommand("Climb Release", m_climber.goToPosition(90));
        NamedCommands.registerCommand("Go Back Coral", m_Intake.backup3inch());
        NamedCommands.registerCommand("Shorter Go Back", m_Intake.backup1andahalfinch());
      

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
        // m_controller.a().onTrue(l1Command());

        // m_controller.

        m_controller.start().onTrue(m_Intake.forward3inch());
        m_controller.back().onTrue(m_Intake.backup3inch());

        m_controller.b().onTrue(l2Command());

        m_controller.y().onTrue(l3Command());

        m_controller.x().onTrue(l4Command());

        m_controller.leftStick().onTrue(m_drivetrain.findAndFollowPath(new Pose2d(5.2619, 4.99953, Rotation2d.fromDegrees(240)))); // 20 Left

        m_controller.rightStick().onTrue(m_drivetrain.findAndFollowPath(new Pose2d(5.2619, 3.05047, Rotation2d.fromDegrees(120)))); // 20 Right        ));

        m_controller.rightBumper().onTrue(algaeclearTop());
                // new Pose2d(4.05, 2.95, Rotation2d.fromDegrees(60)), // 17 Right

        m_controller.leftBumper().onTrue(algaeclearBottom());
        
        m_controller.povRight().onTrue(algueScoreCmd()); 
        
        m_controller.povLeft().onTrue(algaeOutCmd());  
        // m_controller.povLeft().onTrue(m_Intake.reversesame());

        m_controller.povDown().onTrue(m_Intake.stop());

        // m_controller.a().onTrue(m_drivetrain.findAndFollowPath(new Pose2d(7.2, 4.2, Rotation2d.fromDegrees(180))));

        // m_controller.povRight().onTrue(m_drivetrain.AutoAlign(new Pose2d(14.25, 3.8, new Rotation2d(180)), 3));
        // m_controller.povRight().onTrue(m_drivetrain.findAndFollowPath(new Pose2d(6.25, 3.8, Rotation2d.fromDegrees(180))));

        // m_controller.povRight().onTrue(new MoveToPose(new Pose2d(6.8, 3.8, new Rotation2d(180)), m_drivetrain));


        // m_controller.povDown().onTrue(RampRelease());

        // m_controller.a().onTrue(m_Elevator.goToHeight(2));
        // m_controller.y().onTrue(m_Elevator.goToHeight(1));
        // m_controller.rightTrigger().onTrue(m_Elevator.goToHeight(0.05).alongWith(m_Arm.goToAngle(0.26)).alongWith(m_Intake.forwards(true)));

        m_controller.povUp().onTrue(RampRelease(0.5).andThen(ClimberGoUp()));

        m_controller.rightTrigger()
        .whileTrue(ClimberControlLogic())
        .onFalse(m_climber.stop());


        m_controller.leftTrigger()
        .onTrue(m_Arm.goToAngle(loadangle).andThen(m_Intake.forwards(false).withTimeout(1)))
        .onFalse(resetElevatorCmd());

        // m_controller.povDown().onTrue(m_Intake.forwards(true));

        // objectDetected.onTrue(m_Intake.koralControlCommand(0.075)); //-0.38
        // objectDetected.onFalse(m_Intake.forwards(true));
        

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
        
        // m_controller.povUp().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

        // m_drivetrain.registerTelemetry(logger::telemeterize);
    }


    // Pose estimator update logic (meant to increase accuracy by filtering out bad or unusable output from the Cameras)
    public void updatePoseEstimator() {
      rightPoseEstimator.updatePose("Right Side Pose");
      leftPoseEstimator.updatePose("Left Side Pose");
      SmartDashboard.putBoolean("Koral Trigger", !m_koral_sensor.get());


      m_Fieldpose.setRobotPose(m_drivetrain.getPose2d());
      SmartDashboard.putData("RobotPose Field2D", m_Fieldpose);
    
 }



    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }
}
