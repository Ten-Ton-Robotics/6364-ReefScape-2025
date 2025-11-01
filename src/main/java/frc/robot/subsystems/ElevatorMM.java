package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorMM extends SubsystemBase {

    public static final String ELEVATOR_BUS = "drivecan";
    public static final int ELEVATOR_MOTOR_ID_1 = 5;
    public static final int ELEVATOR_MOTOR_ID_2 = 18;

    public static final double ELEVATOR_POSE = 0;
    public static final NeutralModeValue ELEVATOR_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final InvertedValue ELEVATOR_MOTOR_1_INVERTED = InvertedValue.Clockwise_Positive;
    public static final InvertedValue ELEVATOR_MOTOR_2_INVERTED = InvertedValue.Clockwise_Positive;

    public static final double ELEVATOR_KS = 0.0; // output to overcome static friction (output)
    public static final double ELEVATOR_KV = 0.12; // outpt per unit of target velocity (output/rps)
    public static final double ELEVATOR_KA = 0.01; // output per unit of target acceleration (output/(rps/s))
    public static final double ELEVATOR_KP = 225; // output per unit of error in position (output/rotation)
    public static final double ELEVATOR_KI = 0; // output per unit of integrated error in position (output/(rotation*s))
    public static final double ELEVATOR_KD = 12; ///0.30; // output per unit of error in velocity (output/rps)
    public static final double ELEVATOR_KG = 26; // output to overcome gravity (output)
    public static final double CRUISE_VELOCITY = 8; // target cruise velocity (rps)
    public static final double ACCELERATION = 22; // Target acceleration of rps/s (So if target v is 80 and target accel is 160)
    public static final double JERK = 32; // Target jerk rps/s^2
    public static final double CURRENT_LIMIT = 55; // Current Limit
    public static final double ELEVATOR_RATIO = 5.0; // Elevator Ratio
    private final TalonFX m_ElevatorLeader = new TalonFX(ELEVATOR_MOTOR_ID_1, ELEVATOR_BUS);
    private final TalonFX m_ElevatorFollower = new TalonFX(ELEVATOR_MOTOR_ID_2, ELEVATOR_BUS);
    private final MotionMagicTorqueCurrentFOC m_MotionMagicControl = new MotionMagicTorqueCurrentFOC(ELEVATOR_POSE);
 
    public ElevatorMM() {
        super();
        
        final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        // Configure PID gains
        Slot0Configs slot0Configs = elevatorConfig.Slot0;
        slot0Configs.kG = ELEVATOR_KG;
        slot0Configs.kS = ELEVATOR_KS;
        slot0Configs.kV = ELEVATOR_KV;
        slot0Configs.kA = ELEVATOR_KA;
        slot0Configs.kP = ELEVATOR_KP;
        slot0Configs.kI = ELEVATOR_KI;
        slot0Configs.kD = ELEVATOR_KD;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        // Configure Motion Magic settings
        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        elevatorConfig.MotionMagic.MotionMagicAcceleration = ACCELERATION;
        elevatorConfig.MotionMagic.MotionMagicJerk = JERK;

        // Set gearing ratio
        elevatorConfig.Feedback.SensorToMechanismRatio = ELEVATOR_RATIO;

        // Set neutral mode
        m_ElevatorLeader.setNeutralMode(ELEVATOR_NEUTRAL_MODE);
        m_ElevatorFollower.setNeutralMode(ELEVATOR_NEUTRAL_MODE);

        // Set current limit
        elevatorConfig.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Apply configuration to the leader motor
        elevatorConfig.MotorOutput.Inverted = ELEVATOR_MOTOR_1_INVERTED;
        m_ElevatorLeader.getConfigurator().apply(elevatorConfig);
        m_ElevatorLeader.setPosition(ELEVATOR_POSE);

        // Configure follower motors correctly
        m_ElevatorFollower.setControl(new Follower(m_ElevatorLeader.getDeviceID(), false));
    }

    public void setElevatorHeight(double position) {
        m_MotionMagicControl.Position = position;
        m_ElevatorLeader.setControl(m_MotionMagicControl);
    }
    
    public double getElevatorHeight(){
        return m_MotionMagicControl.Position; 
    }


    /**
     * @brief Moves elevator up 
     * 
     * @param target position elvator position 
     * @return command 
     */
    public Command goToHeight(double targetPosition) {
        return this.runOnce(() -> {
            System.out.println("Going to height: " + targetPosition);
            setElevatorHeight(targetPosition);
        });
    }

    /**
     * @brief Stops Elevator Motors 
     * 
     * @return command 
     */
    public Command stop() {
        return this.runOnce(() -> {
            m_ElevatorLeader.setControl(new StaticBrake());
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        builder.addDoubleProperty("Leader Motor Position", () -> m_ElevatorLeader.getPosition().getValueAsDouble(),
            (double position) -> setElevatorHeight(position));

        builder.addDoubleProperty("Leader Motor Target Position", () -> m_MotionMagicControl.Position, (double position) -> setElevatorHeight(position));

        builder.addDoubleProperty("Leader Motor Voltage", () -> m_ElevatorLeader.getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("Follower Motor Voltage", () -> m_ElevatorFollower.getMotorVoltage().getValueAsDouble(), null);

        builder.addDoubleProperty("Leader Motor Amperage", () -> m_ElevatorLeader.getStatorCurrent().getValueAsDouble(), null);
        builder.addDoubleProperty("Follower Motor Amperage", () -> m_ElevatorFollower.getStatorCurrent().getValueAsDouble(), null);

        builder.addDoubleProperty("Motion Magic Running Leader", () ->  m_ElevatorLeader.getMotionMagicIsRunning().getValueAsDouble(), null);
        builder.addDoubleProperty("Motion Magic Running Follower", () ->  m_ElevatorFollower.getMotionMagicIsRunning().getValueAsDouble(), null);

        
    }
}
