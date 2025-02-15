package frc.robot.util;

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
    public static final double kLateralPositionP = 10;
    public static final double kLateralPositionD = 0;
    // angular position controller
    public static final double kAngularPositionP = 0.5;
    public static final double kAngularPositionD = 0;
    // velocity/acceleration constraints
    public static final double kMaxLateralSpeed = 5; // meters per second
    public static final double kMaxLateralAcceleration = 2; // meters per second squared
    public static final double kMaxAngularSpeed = 9.42477796077; // radians per second
    public static final double kMaxAngularAcceleration = 3.06998012384; // radians per second
                                                                        // squared
    // movement tolerances
    public static final double kLateralTolerance = 0.2; // meters
    public static final double kAngularTolerance = 0.2; // radians

    public static final double kBotWidth = 0.7; // meters
    public static final double kBotLength = 1; // meters

    public static TrajectoryConfig K_TRAJECTORY_CONFIG = new TrajectoryConfig(kMaxLateralSpeed, kMaxLateralAcceleration)
        .setKinematics(RobotContainer.m_drivetrain.getKinematics()).setEndVelocity(0);
  }

    
}