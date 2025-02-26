package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorMM extends SubsystemBase {

    public static final String kElevatorBus = "drivecan";
    public static final int kElevatorMotor1Id = 5;
    public static final int kElevatorMotor2Id = 18;

    public static final double kElevatorPose = 0;
    public static final NeutralModeValue kElevatorNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue kElevatorMotor1Inverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kElevatorMotor2Inverted = InvertedValue.Clockwise_Positive;

    public static final double kElevatorKS = 0.0; // output to overcome static friction (output)
    public static final double kElevatorKV = 0.12; // output per unit of target velocity (output/rps)
    public static final double kElevatorKA = 0.01; // output per unit of target acceleration (output/(rps/s))
    public static final double kElevatorKP = 0; // output per unit of error in position (output/rotation)
    public static final double kElevatorKI = 0; // output per unit of integrated error in position (output/(rotation*s))
    public static final double kElevatorKD = 0; // output per unit of error in velocity (output/rps)
    public static final double kElevatorKG = 0; // output to overcome gravity (output)
    public static final double kCruiseVelocity = 80; // target cruise velocity (rps)
    public static final double kAcceleration = 160; // Target acceleration of rps/s (So if target v is 80 and target accel is 160)
    public static final double kJerk = 1600; // 
    public static final double kCurrentLimit = 55;
    public static final double kElevatorRatio = 5.0;

    private final TalonFX m_ElevatorLeader = new TalonFX(kElevatorMotor1Id, kElevatorBus);
    private final TalonFX m_ElevatorFollower = new TalonFX(kElevatorMotor2Id, kElevatorBus);
    private final MotionMagicVoltage m_MotionMagicControl = new MotionMagicVoltage(kElevatorPose);
    private final DutyCycleOut m_request = new DutyCycleOut(0);

    public ElevatorMM() {
        super();
        
        final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        // Configure PID gains
        Slot0Configs slot0Configs = elevatorConfig.Slot0;
        slot0Configs.kG = kElevatorKG;
        slot0Configs.kS = kElevatorKS;
        slot0Configs.kV = kElevatorKV;
        slot0Configs.kA = kElevatorKA;
        slot0Configs.kP = kElevatorKP;
        slot0Configs.kI = kElevatorKI;
        slot0Configs.kD = kElevatorKD;
        slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

        // Configure Motion Magic settings
        elevatorConfig.MotionMagic.MotionMagicCruiseVelocity = kCruiseVelocity;
        elevatorConfig.MotionMagic.MotionMagicAcceleration = kAcceleration;
        elevatorConfig.MotionMagic.MotionMagicJerk = kJerk;

        // Set gearing ratio
        elevatorConfig.Feedback.SensorToMechanismRatio = kElevatorRatio;

        // Set neutral mode
        m_ElevatorLeader.setNeutralMode(kElevatorNeutralMode);
        m_ElevatorFollower.setNeutralMode(kElevatorNeutralMode);

        // Set current limit
        elevatorConfig.CurrentLimits.StatorCurrentLimit = kCurrentLimit;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Apply configuration to the leader motor
        elevatorConfig.MotorOutput.Inverted = kElevatorMotor1Inverted;
        m_ElevatorLeader.getConfigurator().apply(elevatorConfig);
        m_ElevatorLeader.setPosition(kElevatorPose);

        // Configure follower motors correctly
        m_ElevatorFollower.setControl(new Follower(m_ElevatorLeader.getDeviceID(), false));
    }

    public void setElevatorHeight(double position) {
        m_MotionMagicControl.Position = position;
        m_ElevatorLeader.setControl(m_MotionMagicControl);
    }

    public Command goToHeight(double targetPosition) {
        return this.runOnce(() -> {
            System.out.println("Going to height: " + targetPosition);
            setElevatorHeight(targetPosition);
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            m_ElevatorLeader.setControl(m_request.withOutput(0));
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        builder.addDoubleProperty("Leader Motor Position", () -> m_ElevatorLeader.getPosition().getValueAsDouble(),
            (double position) -> setElevatorHeight(position));
    }
}
