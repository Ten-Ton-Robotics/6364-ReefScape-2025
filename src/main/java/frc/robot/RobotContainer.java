// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

<<<<<<< Updated upstream
=======
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

        if (absinput < deadband) {
            return 0;
        }

        return (((Math.pow(a, absinput) * input * inverseA) + s_deadband) * inversemax);
    }

    private double getFieldCentricAngleFromJoystick(final double x, final double y, final double deadzone){
      final double magnitude = Math.hypot(x, y);
      boolean firstrun = true;
      double defaultangle = 0;

      if(firstrun){
        firstrun = false;
        defaultangle = m_drivetrain.getPose2d().getRotation().getRadians();
      }

      if(magnitude > deadzone){

        final double idealy = Math.sqrt((1-Math.pow(x, 2)));
        double rawangle = Math.atan2(idealy, x);

        if(y < 0){
            rawangle = Math.PI + rawangle;
        }

        defaultangle = rawangle;


      }

      return defaultangle;
    }

    
>>>>>>> Stashed changes
    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

<<<<<<< Updated upstream
        drivetrain.registerTelemetry(logger::telemeterize);
=======
        m_controller.x().onTrue(l4Command());

        //m_controller.leftStick().onTrue(m_drivetrain.findAndFollowPath(new Pose2d(5.2619, 4.99953, Rotation2d.fromDegrees(240)))); // 20 Left

        //m_controller.rightStick().onTrue(m_drivetrain.findAndFollowPath(new Pose2d(5.2619, 3.05047, Rotation2d.fromDegrees(120)))); // 20 Right

        m_controller.rightBumper().onTrue(algaeclearTop());
        // new Pose2d(4.05, 2.95, Rotation2d.fromDegrees(60)), // 17 Right

        m_controller.leftBumper().onTrue(algaeclearBottom());
        
        m_controller.povRight().onTrue(algueScoreCmd()); 
        
        m_controller.povLeft().onTrue(algaeOutCmd());  
        // m_controller.povLeft().onTrue(m_Intake.reversesame());

        m_controller.povDown().onTrue(m_Intake.stop());

        m_controller.povUp().onTrue(rampRelease(0.5).andThen(climberGoUp()));

        m_controller.rightTrigger()
        .whileTrue(climberControlLogic())
        .onFalse(m_climber.stop());


        m_controller.leftTrigger()
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
    
>>>>>>> Stashed changes
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
