public class AlignToTag {
    
    private Rotation2d m_target; // the pose to move to
    private final CommandSwerveDrivetrain m_drivetrain; // the drivetrain to move
  
    private final ProfiledPIDController m_angleController; // profiled PID controller for the angle

    public AlignToTag(CommandSwerveDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        m_target = new Rotation2d(0);
        // angular PID + Motion Profile
        m_angleController = new ProfiledPIDController(Drivetrain.kAngularPositionP, 0.0,
            Drivetrain.kAngularPositionD, new TrapezoidProfile.Constraints(Drivetrain.kMaxAngularSpeed,
                Drivetrain.kMaxAngularAcceleration));
    }

    public double getVelocity(){
        
    }
    
  
}
