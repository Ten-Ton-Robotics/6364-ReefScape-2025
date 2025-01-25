package frc.robot.autonomous;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TurnToPose {

  private Rotation2d m_target; // the pose to move to
  private final CommandSwerveDrivetrain m_drivetrain; // the drivetrain to move

  private final ProfiledPIDController m_angleController; // profiled PID controller for the angle

  public TurnToPose(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_target = new Rotation2d(0);
    // angular PID + Motion Profile
    m_angleController = new ProfiledPIDController(Drivetrain.kAngularPositionP, 0.0,
        Drivetrain.kAngularPositionD, new TrapezoidProfile.Constraints(Drivetrain.kMaxAngularSpeed,
            Drivetrain.kMaxAngularAcceleration));
  }

  public double velocity(Rotation2d target) {
    m_target = target;
    // get the current pose of the robot
    final Rotation2d posedif = m_drivetrain.getState().Pose.getRotation().minus(m_target);
    // calculate speeds
    final double angleSpeed = m_angleController.calculate(posedif.getRadians(), 0);
    // move the robot to the target pose
    return angleSpeed;
  }

}
