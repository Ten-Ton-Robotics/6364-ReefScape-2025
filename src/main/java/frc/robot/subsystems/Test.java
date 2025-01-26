package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Test extends SubsystemBase {

    public static final String kUpperMotorBus = "rio";
    public static final String kLowerMotorBus = "rio";
    public static final int kUpperMotorId = 0;
    public static final int kLowerMotorId = 0;
    public static final double kUpperSpeed = 0;
    public static final double kLowerSpeed = 0;
    public static final NeutralModeValue kUpperNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue kLowerNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue kUpperMotorInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kLowerMotorInverted = InvertedValue.CounterClockwise_Positive;
    
    // upper motor controller gains
    public static final double kUpperKP = 1;
    public static final double kUpperKI = 0;
    public static final double kUpperKD = 0;
    // upper motor controller feedforward gains
    public static final double kUpperKS = 0;
    public static final double kUpperKV = 0;
    public static final double kUpperKA = 0;

    // lower motor controller gains
    public static final double kLowerKP = 1;
    public static final double kLowerKI = 0;
    public static final double kLowerKD = 0;
    // lower motor controller feedforward gains
    public static final double kLowerKS = 0;
    public static final double kLowerKV = 0;
    public static final double kLowerKA = 0;

    // Drive Ratio
    public static final double kUpperRatio = 1/9;
    public static final double kLowerRatio = 1/9;

    private final TalonFX m_upperMotor = new TalonFX(kUpperMotorId, kUpperMotorBus);
    private final TalonFX m_lowerMotor = new TalonFX(kLowerMotorId, kLowerMotorBus);
    
    private final VelocityVoltage m_upperOutput = new VelocityVoltage(kUpperSpeed);
    private final VelocityVoltage m_lowerOutput = new VelocityVoltage(kLowerSpeed);

    public Test() {
        super();
        // configure motors
        final TalonFXConfiguration upperConfig = new TalonFXConfiguration();
        final TalonFXConfiguration lowerConfig = new TalonFXConfiguration();
        // set controller gains
        upperConfig.Slot0 = new Slot0Configs().withKP(kUpperKP).withKI(kUpperKI).withKD(kUpperKD)
            .withKS(kUpperKS).withKV(kUpperKV).withKA(kUpperKA);
        lowerConfig.Slot0 = new Slot0Configs().withKP(kLowerKP).withKI(kLowerKI).withKD(kLowerKD)
            .withKS(kLowerKS).withKV(kLowerKV).withKA(kLowerKA);
        // invert motors
        upperConfig.MotorOutput.Inverted = kUpperMotorInverted;
        lowerConfig.MotorOutput.Inverted = kLowerMotorInverted;
        // set ratios
        upperConfig.Feedback.SensorToMechanismRatio = kUpperRatio;
        lowerConfig.Feedback.SensorToMechanismRatio = kLowerRatio;
        // set neutral modes
        m_upperMotor.setNeutralMode(kUpperNeutralMode);
        m_lowerMotor.setNeutralMode(kLowerNeutralMode);
        // apply configuration
        m_upperMotor.getConfigurator().apply((upperConfig));
        m_lowerMotor.getConfigurator().apply((lowerConfig));
        // commands
      }

    public void setUpperSpeed(double speed) {
    m_upperOutput.Velocity = speed;
    m_upperMotor.setControl(m_upperOutput);
    m_upperMotor.setNeutralMode(NeutralModeValue.Brake);
    if (speed == 0.0)
      m_lowerMotor.setControl(new StaticBrake());
  }

  /**
   * @brief set the speed of the lower motor
   * 
   * @param speed speed in revolutions per second
   * @return Command
   */
  public void setLowerSpeed(double speed) {
    m_lowerOutput.Velocity = speed;
    m_lowerMotor.setControl(m_lowerOutput);
    m_lowerMotor.setNeutralMode(NeutralModeValue.Brake);
    if (speed == 0.0)
      m_lowerMotor.setControl(new StaticBrake());
  }

  /**
   * @brief set the speed of the flywheel
   * 
   * @param speed speed in revolutions per second
   * @return Command
   */
  public Command setSpeed(double speed) {
       return this.runOnce(() -> {
         this.setUpperSpeed(speed);
         this.setLowerSpeed(speed);
       });
     }
   


    public Command reverse() {
            return this.runOnce(() -> {
            this.setUpperSpeed(-kUpperSpeed);
            this.setLowerSpeed(-kLowerSpeed);
            });
    }

  /**
   * @brief Spin up the flywheel motors
   * 
   * @return Command
   */
  public Command forwards() {
    return this.runOnce(() -> {
      this.setUpperSpeed(kUpperSpeed);
      this.setLowerSpeed(kLowerSpeed);
    });
  }

  /**
   * @brief Stop the flywheel motors
   * 
   * @return Command
   */
  public Command stop() {
    return this.setSpeed(0);
  }


  /**
   * @brief Send telemetry data to Shuffleboard
   * 
   *        The SendableBuilder object is used to send data to Shuffleboard. We use it to send the
   *        target velocity of the motors, as well as the measured velocity of the motors. This
   *        allows us to tune intake speed in real time, without having to re-deploy code.
   * 
   * @param builder the SendableBuilder object
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder); // call the superclass method
    // add upper motor target velocity property
    // builder.addDoubleProperty("Upper Target Velocity", () -> m_upperOutput.Velocity,
    //     (double target) -> this.setUpperSpeed(target));
    // // add upper motor measured velocity property
    // builder.addDoubleProperty("Upper Measured Velocity",
    //     () -> m_upperMotor.getVelocity().getValueAsDouble(), null);
    // // add lower motor target velocity property
    // builder.addDoubleProperty("Lower Target Velocity", () -> m_lowerOutput.Velocity,
    //     (double target) -> this.setLowerSpeed(target));
    // // add lower motor measured velocity property
    // builder.addDoubleProperty("Lower Measured Velocity",
    //     () -> m_lowerMotor.getVelocity().getValueAsDouble(), null);
  }

}
