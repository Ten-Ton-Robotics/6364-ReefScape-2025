package frc.robot.autonomous;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.Constants.Drivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

/**
 * A command that moves the robot to a specified pose
 */
public class MoveToPose extends Command {
  private final Pose2d m_target; // the pose to move to
  private final CommandSwerveDrivetrain m_drivetrain; // the drivetrain to move

  private final ProfiledPIDController m_angleController; // profiled PID controller for the angle
  private final ProfiledPIDController m_xController; // PID controller for the X position of the
                                                     // robot
  private final ProfiledPIDController m_yController; // PID controller for the Y position of the
                                                     // robot

  private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
      .withDriveRequestType(DriveRequestType.Velocity); // field-centric

  /**
   * @brief motion to move the robot to a specified pose
   * 
   *        Unlike most motions in autonomous, this motion is not generated before
   *        runtime This is
   *        to enable the robot to move to a pose that is not known until runtime
   * 
   *        This algorithm uses a trapezoidal, time-based motion profile to move
   *        the robot to the
   *        specified pose along with a PID controller for feedback control
   * 
   *        The path the robot follows is a straight line from the robot's current
   *        pose to the
   *        specified pose
   * 
   * @param pose       the pose to move to
   * @param drivetrain the drivetrain to move
   */
  public  MoveToPose(Pose2d target, CommandSwerveDrivetrain drivetrain) {
    m_target = target;
    m_drivetrain = drivetrain;
    // angular PID + Motion Profile
    m_angleController = new ProfiledPIDController(Drivetrain.kAngularPositionP, 0.0,
        Drivetrain.kAngularPositionD, new TrapezoidProfile.Constraints(Drivetrain.kMaxAngularSpeed,
            Drivetrain.kMaxAngularAcceleration));
    m_angleController.setTolerance(Drivetrain.kAngularTolerance);
    // x PID + Motion Profile
    m_xController = new ProfiledPIDController(Drivetrain.kLateralPositionP, 0.0,
        Drivetrain.kLateralPositionD, new TrapezoidProfile.Constraints(Drivetrain.kMaxLateralSpeed,
            Drivetrain.kMaxLateralAcceleration));
    m_xController.setTolerance(Drivetrain.kLateralTolerance);
    // y PID + Motion Profile
    m_yController = new ProfiledPIDController(Drivetrain.kLateralPositionP, 0.0,
        Drivetrain.kLateralPositionD, new TrapezoidProfile.Constraints(Drivetrain.kMaxLateralSpeed,
            Drivetrain.kMaxLateralAcceleration));
    m_yController.setTolerance(Drivetrain.kLateralTolerance);
  }

  /**
   * @brief runs periodically while the command is scheduled
   * 
   */
  @Override
  public void execute() {
    // get the current pose of the robot
    final Pose2d pose = m_drivetrain.getState().Pose;
    // calculate speeds
    double xSpeed = m_xController.calculate(pose.getTranslation().getX(), m_target.getTranslation().getX());
    double ySpeed = m_yController.calculate(pose.getTranslation().getY(), m_target.getTranslation().getY());
    double angleSpeed = m_angleController.calculate(pose.getRotation().getRadians(),
        m_target.getRotation().getRadians());

    // move the robot to the target pose
    m_drivetrain.applyRequest(
        () -> m_drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(-angleSpeed));
    m_drivetrain.setControl(
        m_drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(-angleSpeed));
  }

  /**
   * @brief checks if the command is finished
   * 
   * @return true if the robot is at the target pose, false otherwise
   */
  @Override
  public boolean isFinished() {
    return m_xController.atGoal() && m_yController.atGoal() && m_angleController.atGoal();
  }

  /**
   * @brief ends the command
   * 
   */
  @Override
  public void end(boolean interrupted) {
    if (!interrupted)
      m_drivetrain
          .applyRequest(() -> m_drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }
};
