package frc.robot.util;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.RobotContainer;

public class Constants {
      public class Drivetrain {
    // translation sysid constants
    public static final double kTranslationalRampRate = 0.5;
    public static final double kTranslationalStepVoltage = 4.0;
    public static final double kTranslationalTimeout = 15.0;
    // steer sysid constants 
    public static final double kSteerRampRate = 0.5;
    public static final double kSteerStepVoltage = 7.0;
    public static final double kSteerTimeout = 15.0;
    // rotational sysid constants
    public static final double kRotationRampRate = 0.2;
    public static final double kRotationStepVoltage = 3.0;
    public static final double kRotationTimeout = 15.0;
    // lateral position controller
    public static final double kLateralPositionP = 15;
    public static final double kLateralPositionD = 0;
    // angular position controller
    public static final double kAngularPositionP = 5;
    public static final double kAngularPositionD = 0; 
    // velocity/acceleration constraints
    public static final double kMaxLateralSpeed = 2.0; // meters per second
    public static final double kMaxLateralAcceleration = 1.0; // meters per second squared
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kMaxAngularAcceleration = 1.0 * Math.PI; // radians per second
                                                                        // squared
    // movement tolerances
    public static final double kLateralTolerance = 0.02; // meters
    public static final double kAngularTolerance = 0.02; // radians

    public static final double kBotWidth = 0.927; // meters
    public static final double kBotLength = 0.927; // meters

    public static TrajectoryConfig K_TRAJECTORY_CONFIG = new TrajectoryConfig(kMaxLateralSpeed, kMaxLateralAcceleration)
        .setKinematics(RobotContainer.m_drivetrain.getKinematics()).setEndVelocity(0);
  }

    public static class Swerve {
        public static enum REEFS {
            LEFT, RIGHT
        }

        public static final List<Pose2d> STATION_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(1.151, 1.03, Rotation2d.fromDegrees(55)), // 12 Station
                new Pose2d(1.1383, 7.01, Rotation2d.fromDegrees(-55)) // 13 Station 1.0873
            )
        );

        public static final List<Pose2d> LEFT_REEF_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(3.70, 3.16, Rotation2d.fromDegrees(60)), // 17 Left
                new Pose2d(3.30, 4.15, Rotation2d.fromDegrees(0)), // 18 Left
                new Pose2d(4.05, 5.1, Rotation2d.fromDegrees(300)), // 19 Left
                new Pose2d(5.2619, 4.99953, Rotation2d.fromDegrees(240)), // 20 Left
                new Pose2d(5.70, 3.85, Rotation2d.fromDegrees(180)), // 21 Left
                new Pose2d(4.9113, 2.93927, Rotation2d.fromDegrees(120)) // 22 Left
            )
        );

        public static final List<Pose2d> RIGHT_REEF_WAYPOINTS = new ArrayList<Pose2d>(
            List.of(
                new Pose2d(4.05, 2.95, Rotation2d.fromDegrees(60)), // 17 Right
                new Pose2d(3.30, 3.85, Rotation2d.fromDegrees(0)), // 18 Right
                new Pose2d(3.70, 4.89, Rotation2d.fromDegrees(300)), // 19 Right
                new Pose2d(4.9419, 5.16453, Rotation2d.fromDegrees(240)), // 20 Right
                new Pose2d(5.70, 4.20, Rotation2d.fromDegrees(180)), // 21 Right
                new Pose2d(5.2619, 3.05047, Rotation2d.fromDegrees(120)) // 22 Right
            )
        );
    }


    
}