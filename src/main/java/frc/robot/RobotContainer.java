// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;  
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorMM;
import frc.robot.subsystems.Intake;
import frc.robot.util.PhotonVisionHandler;
import frc.robot.util.PoseEstimatorInst;

public class RobotContainer {
    //Misc Objects 
    private final SendableChooser<Command> autoChooser;
    
    //Sub-Objects 
    public final Climb m_climber = new Climb();
    public final Intake m_Intake = new Intake();
    public static final Arm m_Arm = new Arm(); 
    public final ElevatorMM m_Elevator = new ElevatorMM();
    public static final DigitalInput m_CoralSensor = new DigitalInput(0);
    public static final Servo m_RampRelease1 = new Servo(1); 
    public static final Servo m_RampRelease2 = new Servo(2); 

    //Variables for subobjects
    Trigger objectDetected = new Trigger(() -> !m_CoralSensor.get());
    
    //Variables for vision/pose 
    private final PoseEstimatorInst rightPoseEstimator;
    private final PoseEstimatorInst leftPoseEstimator;
    private final Field2d m_VisionPoseFront = new Field2d();
    private final Field2d m_VisionPoseBack = new Field2d();
    private final Field2d m_FieldPose = new Field2d();

    //Left Camera Offsets 
    private final Transform3d robotToCamLeft =
      new Transform3d(new Translation3d(Units.inchesToMeters(8), Units.inchesToMeters(13),
          Units.inchesToMeters(13.50)), new Rotation3d(0, 0, Math.toRadians(-30))); // Adjusted

    //Right Camera Offsets 
    private final Transform3d robotToCamRight =
          new Transform3d(new Translation3d(Units.inchesToMeters(8), Units.inchesToMeters(-13),
              Units.inchesToMeters(13.50)), new Rotation3d(0, 0, Math.toRadians(30))); // Adjusted
        
    public final PhotonVisionHandler visionHandlerLeft = new PhotonVisionHandler("Back", robotToCamLeft);
    public final PhotonVisionHandler visionHandlerRight = new PhotonVisionHandler("Front", robotToCamRight);

    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    

    //Variables for Driving 
    private static final double MAX_ANGULAR_RATE = 1.0; //Radians/s 
    private static final double MAX_SPEED = 1.0;

    // private static final double kAngulardeadband = kMaxAngularRate * 0.1;
    // private static final double kLineardeadband = kMaxSpeed * 0.1;

    // private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric();
    //     .withDeadband(kLineardeadband)
    // .withRotationalDeadband(kAngulardeadband) // 20% deadband
    // .withDriveRequestType(DriveRequestType.Velocity); // closed loop velocity control
    
    private final SwerveRequest.FieldCentricFacingAngle m_drive_new = new SwerveRequest.FieldCentricFacingAngle()
    .withDriveRequestType(DriveRequestType.Velocity); // closed loop velocity control

    /* Setting up bindings for necessary control of the swerve drive platform */

    private final Telemetry logger = new Telemetry(MAX_SPEED);
    private double loadangle = 0.26;

    public static final CommandXboxController m_controller = new CommandXboxController(0);

    public final static CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    public void init(){
      objectDetected.onTrue(m_Intake.coralControlCommand(0.055)); //-0.38
      objectDetected.onFalse(m_Intake.forwards(true));

      m_Arm.goToAngle(0.26).schedule();

      if (!objectDetected.getAsBoolean()) {
        m_Intake.forwards(true).alongWith(m_Arm.goToAngle(0.26)).schedule();
      }
    }
 
    private double expoCurve(final double input, final double a, final double deadband) {
        final double absinput = Math.abs(input);
        final double inverseA = (1.0 / a);
        final double s_deadband = (deadband * Math.signum(input));
        final double inversemax = 1.0 / (1.0 + deadband);
        double elevatorHeightDampener = 1; 
        if (absinput < deadband) {
            return 0;
        }

        if(m_Elevator.getElevatorHeight() > 1){
          elevatorHeightDampener = (1/m_Elevator.getElevatorHeight()); 
        }


        return (((Math.pow(a, absinput) * input * inverseA) + s_deadband) * inversemax * elevatorHeightDampener);
    }

    private Rotation2d AngleGetter(final double x, final double y, final double deadzone){
      final double magnitude = Math.hypot(x, y);
      Rotation2d angle; 

      if(magnitude > deadzone){

        angle = new Rotation2d(Math.atan2(y*-1, x) - Math.toRadians(90));  
      }
      else{
        angle = m_drivetrain.getPose2d().getRotation(); 
      }
      return angle;
    }

    private double maxRate(final double x, final double y, final double deadzone){
      final double magnitude = Math.hypot(x, y); 
      if(magnitude > deadzone){
        return magnitude * 20; //20 is the recommended max PID  
      }else{
        return 0; 
      }
    }

    

    public RobotContainer() {
        configureBindings();

        m_RampRelease1.set(0);
        m_RampRelease2.set(0);


        rightPoseEstimator = new PoseEstimatorInst(visionHandlerRight, m_drivetrain, m_VisionPoseFront);
        leftPoseEstimator = new PoseEstimatorInst(visionHandlerLeft, m_drivetrain, m_VisionPoseBack);
  
        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData("Elevator", m_Elevator);

        //-------------------------------
        // UNCOMMENT FOR DIAGNOSTICS     
        //-------------------------------
        
        // SmartDashboard.putData("Arm", m_Arm);      
        // SmartDashboard.putData("intake", m_Intake);
        // SmartDashboard.putData("climb", m_climber);

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

    private Command l4Command(){
      return new SequentialCommandGroup(
        m_Arm.goToAngle(0.26 * 0.67).withTimeout(1.5),
        m_Elevator.goToHeight(4.955),
        new WaitCommand(1),
        new InstantCommand(() -> loadangle = 0.26 * 0.67),
        m_Intake.stop()
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
      );
    }

    private Command algueScoreCmd(){
      return new SequentialCommandGroup(
        m_Intake.forwardsame(),
        m_Elevator.goToHeight(1.1),
        m_Arm.goToAngle(-0.05) 
      );
    }

    private Command outTakeCmd(){
      return new SequentialCommandGroup(
        m_Arm.goToAngle(loadangle),
        m_Intake.forwards(false).withTimeout(1),
        m_Intake.forwards(true)
      );
    }

    private Command waituntilCoral(){
      return new SequentialCommandGroup(
      Commands.waitUntil(() -> objectDetected.getAsBoolean()),
      new WaitCommand(0.075),
      m_Intake.stop()
      );
    }

    private Command climberControlLogic(){
      return new InstantCommand(() ->{
            m_climber.setClimbVoltage(10);
      });
    }

    private Command climberGoUp(){
      return new InstantCommand(() ->{
            m_climber.goToPosition(90).schedule();
      });
    }

    private Command rampRelease(double val){
      return new InstantCommand(() ->{
        m_RampRelease1.set(val);
        m_RampRelease2.set(val);
      });
    }
    
    private void configureBindings() {
        rampRelease(1);

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(() -> m_drive_new.withVelocityX(-expoCurve(m_controller.getLeftY(), 20, 0.1) * MAX_SPEED)
            .withVelocityY(-expoCurve(m_controller.getLeftX(), 20, 0.1) * MAX_SPEED)
            .withTargetDirection(AngleGetter(m_controller.getRightX(), m_controller.getRightY() , 0.3))
            .withHeadingPID(maxRate(m_controller.getRightX(), m_controller.getRightY(), 0.3), 0, 0)
            .withMaxAbsRotationalRate(MAX_ANGULAR_RATE)
            )
        );

        NamedCommands.registerCommand("Wait For Coral", waituntilCoral());
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
      
        m_controller.start().onTrue(m_Intake.forward3inch());
        m_controller.back().onTrue(m_Intake.backup3inch());

        m_controller.b().onTrue(l2Command());

        m_controller.y().onTrue(l3Command());

        m_controller.x().onTrue(l4Command());

        m_controller.leftStick().onTrue(m_drivetrain.findAndFollowPath(new Pose2d(5.2619, 4.99953, Rotation2d.fromDegrees(240)))); // 20 Left

        m_controller.rightStick().onTrue(m_drivetrain.findAndFollowPath(new Pose2d(5.2619, 3.05047, Rotation2d.fromDegrees(120)))); // 20 Right

        //m_controller.rightBumper().onTrue(algaeclearTop());

        //m_controller.leftBumper().onTrue(algaeclearBottom());



        //m_controller.rightStick().onTrue(m_drivetrain.findAndFollowPath(new Pose2d(5.2619, 3.05047, Rotation2d.fromDegrees(120)))); // 20 Right

        // new Pose2d(4.05, 2.95, Rotation2d.fromDegrees(60)), // 17 Right
        
        
        // new Pose2d(4.05, 2.95, Rotation2d.fromDegrees(60)), // 17 Right
        
        m_controller.povRight().onTrue(algueScoreCmd()); 
        
        m_controller.povLeft().onTrue(algaeOutCmd());  
        
        // m_controller.povLeft().onTrue(m_Intake.reversesame());

        m_controller.povDown().onTrue(m_Intake.stop());

        m_controller.povUp().onTrue(rampRelease(0.5).andThen(climberGoUp()));

        m_controller.leftTrigger()
        .whileTrue(climberControlLogic())
        .onFalse(m_climber.stop());

        m_controller.rightTrigger()
        .onTrue(m_Arm.goToAngle(loadangle).andThen(m_Intake.forwards(false).withTimeout(1)))
        .onFalse(resetElevatorCmd());

        // CTRE Logger
        m_drivetrain.registerTelemetry(logger::telemeterize);

   }


    // Pose estimator update logic (meant to increase accuracy by filtering out bad or unusable output from the Cameras)
    public void updatePoseEstimator() {
      rightPoseEstimator.updatePose("Right Side Pose");
      leftPoseEstimator.updatePose("Left Side Pose");
      SmartDashboard.putBoolean("Coral Trigger", objectDetected.getAsBoolean());

      // Display Robot Pose on shuffleboard
      m_FieldPose.setRobotPose(m_drivetrain.getPose2d());
      SmartDashboard.putData("RobotPose Field2D", m_FieldPose);
      
      SmartDashboard.putNumber("Heading", new Rotation2d(Math.atan2(m_controller.getRightY(), m_controller.getRightX())).getDegrees());
      SmartDashboard.putNumber("Right Joy Stick X", m_controller.getRightX()); 
    }

    // Command to return the result of the autochooser to select the autonomous
    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }

    // Function Wrapper to externalize the odometry thread startup without using public variables
    public void startOdometryThread() {
      m_drivetrain.getOdometryThread().start();
    }
}
