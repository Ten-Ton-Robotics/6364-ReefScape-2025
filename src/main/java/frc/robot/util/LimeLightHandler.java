// package frc.robot.util;

// import edu.wpi.first.networktables.DoubleArraySubscriber;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.TimestampedDoubleArray;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.util.Units;

// /**
//  * @brief LimeLight wrapper
//  *
//  *        We interact with the limelight through networktables. It posts data, and we need to read
//  *        that data from networktables.
//  * 
//  *        Using networktables all the time inflates code size, so we have this wrapper to simplify
//  *        using limelights
//  */
// public class LimeLightHandler {
//   public class MeasurementInfo {
//     public final Integer tagId;
//     public final Integer tagCount;
//     public final Double tagArea;

//     public MeasurementInfo(Integer tagId, Integer tagCount, Double tagArea) {
//       this.tagId = tagId;
//       this.tagCount = tagCount;
//       this.tagArea = tagArea;
//     }
//   }

//   public Boolean isConnected() {
//     return m_table.containsSubTable("botpose_wpiblue");
//   }

//   private DoubleArraySubscriber m_poseSubscriber; // red pose subscriber
//   private final NetworkTable m_table; // limelight network table instance

//   /**
//    * @brief Vision class constructor
//    * 
//    * @param limelightName name of the limelight
//    */
//   public LimeLightHandler(String limelightName) {
//     m_table = NetworkTableInstance.getDefault().getTable(limelightName);
//   }

//   /**
//    * @brief initialize the limelight
//    * 
//    *        This is not run in the constructor because it is not safe to run networktables during
//    *        program startup, as the networktables server may not be running yet.
//    */
//   public void init() {
//     // robot position is different if its on the Blue alliance or the Red alliance
//     if (isConnected()) {
//       m_poseSubscriber = m_table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[7]);
//     }
//   }

//   /**
//    * @brief get the number of tags in view
//    * 
//    * @return double number of tags in view
//    */
//   private Integer numTags() {
//     final double[] cornerCount = m_table.getEntry("tcornxy").getDoubleArray(new double[32]);
//     return (int) Math.ceil(cornerCount.length / 8);
//   }

//   /**
//    * @brief get the area of the detected tag in mm^2
//    * 
//    * @return double area of the tag in mm^2
//    */
//   private double tagArea() {
//     return m_table.getEntry("ta").getDouble(0.0);
//   }

//   /**
//    * @brief get information about the detected tag
//    * 
//    * @return MeasurementInfo information about the detected tag
//    */
//   public MeasurementInfo tagDetector() {
//     final Integer tagID = Math.toIntExact(m_table.getEntry("tid").getInteger(-1));
//     return new MeasurementInfo(tagID, numTags(), tagArea());
//   }

//   /**
//    * @brief get the 2D position measured by the limelight
//    * 
//    * @return Pose2d 2D position measured by the limelight
//    */
//   public Pose2d getPos2D() {
//     try {
//       final double[] raw = m_poseSubscriber.get();
//       return new Pose2d(new Translation2d(raw[0], raw[1]),
//           new Rotation2d(Units.degreesToRadians(raw[5])));
//     } catch (NullPointerException e) {
//       System.out.println(e.toString());
//       return null;
//     }
//   }

//   /**
//    * @brief get the distance from the robot to the tag
//    * 
//    * @return distance in meters
//    */
//   public double getDist3D() {
//     // get the measured pose in the target coordinate system
//     final double[] measuredPoseArray =
//         m_table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
//     // create the vector
//     final Translation3d measuredPose =
//         new Translation3d(measuredPoseArray[0], measuredPoseArray[1], measuredPoseArray[2]);
//     // return the magnitude of the vector
//     return measuredPose.getNorm();
//   }

//   /**
//    * @brief get the timestamp of the latest measurement in milliseconds
//    * 
//    * @return long timestamp of the latest measurement in milliseconds
//    */
//   public long getLatestTimestamp() {
//     return m_poseSubscriber.getAtomic().timestamp;
//   }

//   /**
//    * @brief get the raw pose data
//    * 
//    * @return TimestampedDoubleArray raw pose data
//    */
//   public TimestampedDoubleArray getPoseRaw() {
//     return m_poseSubscriber.getAtomic();
//   }

//   /**
//    * @brief get the latest latency adjusted timestamp in seconds
//    * 
//    * @return double latest latency adjusted timestamp in seconds
//    */
//   public double getLatestLatencyAdjustedTimeStamp() {
//     final TimestampedDoubleArray internal2 = m_poseSubscriber.getAtomic();
//     return ((internal2.timestamp - internal2.value[6]) / 1000.0);
//   }
// }
