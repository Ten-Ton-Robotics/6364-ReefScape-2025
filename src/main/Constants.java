public class Constants {

    public class PathPlanner {
        public static final PIDConstants kTranslationalPIDConstants = new PIDConstants(0.0, 0.0, 0.0);
        public static final PIDConstants kRotationalPIDConstants = new PIDConstants(3.0, 0.0, 0.0);
        public static final double kMaxModuleSpeed = 4.5; // 4.5 m/s.
        public static final double kDriveBaseRadius = 0.4; // 0.4 m. Distance from robot center to
                                                           // furthest module.
        public static final ReplanningConfig kReplanningConfig = new ReplanningConfig(true, true);
      }
    
    
}
