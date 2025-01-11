// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {

        // 6 meters per second desired top speed.
    private static final double kMaxSpeed = 6;

    // Half a rotation per second max angular velocity.
    private static final double kMaxAngularRate = 1.5 * Math.PI;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    private final SwerveRequest.FieldCentricFacingAngle m_angleRequest = new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(kMaxSpeed * 0.05)
    .withRotationalDeadband(kMaxAngularRate * 0.05)
    .withDriveRequestType(DriveRequestType.Velocity);

    private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
    .withDeadband(kMaxSpeed * 0.05).withRotationalDeadband(kMaxAngularRate * 0.05) // 20% deadband
    .withDriveRequestType(DriveRequestType.Velocity); // closed loop velocity control


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_controller = new CommandXboxController(0);

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
    }
    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.applyRequest(() -> m_drive.withVelocityX(getLeftY() * kMaxSpeed)
            .withVelocityY(getLeftX() * kMaxSpeed)
            .withRotationalRate(-m_controller.getRightX() * kMaxAngularRate));


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

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
