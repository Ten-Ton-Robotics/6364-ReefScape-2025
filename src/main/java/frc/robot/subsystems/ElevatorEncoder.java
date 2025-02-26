package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorEncoder extends SubsystemBase {

    public static final String kElevatorBus = "drivecan";
    public static final int kElevatorMotor1Id = 5;
    public static final int kElevatorMotor2Id = 18;

    public static final double kElevatorPose = 0;
    public static final NeutralModeValue kElevatorNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue kElevatorMotor1Inverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kElevatorMotor2Inverted = InvertedValue.Clockwise_Positive;

    // Elevator controller gains for Going Up
    public static final double kElevatorKP = 14;
    public static final double kElevatorKI = 0;
    public static final double kElevatorKD = 0;
    public static final double kElevatorKG = 0.4;

    // Elevator controller gains for Going Up
    public static final double kElevatorDownKP = 2;
    public static final double kElevatorDownKI = 0;
    public static final double kElevatorDownKD = 1;
    public static final double kElevatorDownKG = 0.4;

    public static final double kCurrentLimit = 55;
    public static final double kMaxPosition = 4.5; // (ACTUAL MAX HEIGHT)
    public static final double kElevatorRatio = 5.0; // (ACTUAL GEAR RATIO!)

    private final TalonFX m_ElevatorMotor1 = new TalonFX(kElevatorMotor1Id, kElevatorBus);
    private final TalonFX m_ElevatorMotor2 = new TalonFX(kElevatorMotor2Id, kElevatorBus);
    private final PositionVoltage m_PositionControl = new PositionVoltage(kElevatorPose);

    public ElevatorEncoder() {

        super();
        final TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();

        // Set controller gains
        elevatorConfig.Slot0 = new Slot0Configs().withKP(kElevatorKP).withKI(kElevatorKI).withKD(kElevatorKD).withKG(kElevatorKG).withGravityType(GravityTypeValue.Elevator_Static);
        //For descending 
        elevatorConfig.Slot1 = new Slot1Configs().withKP(kElevatorDownKP).withKI(kElevatorDownKI).withKD(kElevatorDownKD).withKG(kElevatorDownKG).withGravityType(GravityTypeValue.Elevator_Static);

        // Set gearing ratio
        elevatorConfig.Feedback.SensorToMechanismRatio = kElevatorRatio;
        
        // Set neutral mode
        m_ElevatorMotor1.setNeutralMode(kElevatorNeutralMode);
        m_ElevatorMotor2.setNeutralMode(kElevatorNeutralMode);

        // Set current limit
        elevatorConfig.CurrentLimits.StatorCurrentLimit = kCurrentLimit;
        elevatorConfig.CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
        elevatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elevatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Apply inversion settings
        elevatorConfig.MotorOutput.Inverted = kElevatorMotor1Inverted;
        m_ElevatorMotor1.getConfigurator().apply(elevatorConfig);
        m_ElevatorMotor1.setPosition(kElevatorPose);

        elevatorConfig.MotorOutput.Inverted = kElevatorMotor2Inverted;
        m_ElevatorMotor2.getConfigurator().apply(elevatorConfig);
        m_ElevatorMotor2.setPosition(kElevatorPose);
    }

    public void setElevatorHeight(double position) {
        m_PositionControl.Position = position;
        m_PositionControl.Slot = 0;
        m_ElevatorMotor1.setControl(m_PositionControl);
        m_ElevatorMotor2.setControl(m_PositionControl);
    }

    public void decend(double position) {
        m_PositionControl.Position = position;
        m_PositionControl.Slot = 1;
        m_ElevatorMotor1.setControl(m_PositionControl);
        m_ElevatorMotor2.setControl(m_PositionControl);
    }

    public Command goToHeight(double targetPosition) {
        return this.runOnce(() -> {
            System.out.println("going to height: " + targetPosition);
            setElevatorHeight(targetPosition);
        });
    }

    public Command goBackDown(double targetPosition) {
        return this.runOnce(() -> {
            System.out.println("going to down to: " + targetPosition);
            decend(targetPosition);
        });
    }

    public Command stop() {
        return this.runOnce(() -> {
            m_ElevatorMotor1.setControl(new StaticBrake());
            m_ElevatorMotor2.setControl(new StaticBrake());
        });
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        builder.addDoubleProperty("Motor 1 Position", () -> m_ElevatorMotor1.getPosition().getValueAsDouble(),
            (double position) -> setElevatorHeight(position));

        builder.addDoubleProperty("Motor 2 Position", () -> m_ElevatorMotor2.getPosition().getValueAsDouble(),
            (double position) -> setElevatorHeight(position));

        builder.addDoubleProperty("Target Position", () -> m_PositionControl.Position,
            (double target) -> this.goToHeight(target).schedule());
    }
}
