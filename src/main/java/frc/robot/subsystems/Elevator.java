package frc.robot.subsystems;
import java.io.Console;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

    public static final String kElevatorBus = "rio";
    public static final int kElevatorMotor1Id = 0; // not set, change to actual id
    public static final int kElevatorMotor2Id = 0; // not set, change to actual id

    public static final double kElevatorPose = 0;
    public static final NeutralModeValue kElevatorNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue kElevatorInverted = InvertedValue.CounterClockwise_Positive;
    
    // Elevator controller gains
    public static final double kElevatorKP = 50;
    public static final double kElevatorKI = 0;
    public static final double kElevatorKD = 0;

    // Elevator controller feedforward gains
    public static final double kElevatorKG = 0;
    public static final double kElevatorKS = 0;
    public static final double kElevatorKV = 0;
    public static final double kElevatorKA = 0;
    
    public static final double kCurrentLimit = 10;

    // Elevator Pose
    public static final double kMaxPosition = 0;

    // Drive Ratio
    public static final double kElevatorRatio = 0; // not set, change to actual ratio

    private final TalonFX m_ElevatorMotor1 = new TalonFX(kElevatorMotor1Id, kElevatorBus);
    private final TalonFX m_ElevatorMotor2 = new TalonFX(kElevatorMotor2Id, kElevatorBus);

    
    private final PositionVoltage m_ElevatorOutput = new PositionVoltage(kElevatorPose);

    public Elevator() {
        super();
        // Configure Elevator
        final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration(); 

        // Set controller gains
        elevatorConfig.Slot0 = new Slot0Configs().withKP(kElevatorKP).withKI(kElevatorKI).withKD(kElevatorKD)
            .withKS(kElevatorKS).withKV(kElevatorKV).withKA(kElevatorKA).withKG(kElevatorKG);
        
        // Invert motor 
        elevatorConfig.MotorOutput.Inverted = kElevatorInverted; 

        // Set ratios 
        elevatorConfig.Feedback.SensorToMechanismRatio = kElevatorRatio; 
        
        // Set neutral modes 
        m_ElevatorMotor1.setNeutralMode(kElevatorNeutralMode);
        m_ElevatorMotor2.setNeutralMode(kElevatorNeutralMode);

        // Set current limit
        elevatorConfig.CurrentLimits.StatorCurrentLimit = kCurrentLimit;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    
        // Apply Configs 
        m_ElevatorMotor1.getConfigurator().apply(elevatorConfig); 
        m_ElevatorMotor1.setPosition(kMaxPosition);

        m_ElevatorMotor2.getConfigurator().apply(elevatorConfig); 
        m_ElevatorMotor2.setPosition(kMaxPosition);

      }

    public void setElevatorSpeed(double speed) {
        m_ElevatorOutput.Velocity = speed;
        m_ElevatorMotor1.setControl(m_ElevatorOutput);
        m_ElevatorMotor2.setControl(m_ElevatorOutput);

        m_ElevatorMotor1.setNeutralMode(NeutralModeValue.Brake);
        m_ElevatorMotor2.setNeutralMode(NeutralModeValue.Brake);

        if (speed == 0.0){
            m_ElevatorMotor1.setControl(new StaticBrake());
            m_ElevatorMotor2.setControl(new StaticBrake());
        }
    }

    /**
     * @brief Set the speed of the elevator
     * 
     * @param speed Speed in revolutions per second
     * @return Command
     */
    public Command setSpeed(double speed) {
       return this.runOnce(() -> {
         this.setElevatorSpeed(speed);
       });
    }

    /**
     * @brief Stop the elevator
     * 
     * @return Command
     */
    public Command stop() {
        return this.setSpeed(0);
    }

    public Command goToHeight(double position) {
        return this.runOnce(() -> {
            m_ElevatorOutput.Position = position;
            m_ElevatorMotor1.setControl(m_ElevatorOutput);
            m_ElevatorMotor2.setControl(m_ElevatorOutput);
        });
    }

    /**
     * @brief Send telemetry data to Shuffleboard
     * 
     * @param builder The SendableBuilder object
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        builder.addDoubleProperty("Motor 1 Position", () -> m_ElevatorMotor1.getPosition().getValueAsDouble(),
        (double position) -> m_ElevatorMotor1.setPosition(position));
        builder.addDoubleProperty("Motor 2 Position", () -> m_ElevatorMotor2.getPosition().getValueAsDouble(),
        (double position) -> m_ElevatorMotor2.setPosition(position));
        builder.addDoubleProperty("Target Position", () -> m_ElevatorOutput.Position,
        (double target) -> this.goToHeight(target).schedule());
    }
}
