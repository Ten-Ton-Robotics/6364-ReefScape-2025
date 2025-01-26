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

public class Arm extends SubsystemBase {

    public static final String kArmBus = "rio";
    public static final int kArmId = 3;
    public static final double kArmSpeed = 10;
    public static final NeutralModeValue kArmNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue kArmInverted = InvertedValue.CounterClockwise_Positive;
    
    //arm controller gains
    public static final double kArmKP = 0.35;
    public static final double kArmKI = 0;
    public static final double kArmKD = 0;
    //arm controller feedforward gains
    public static final double kArmKS = 0;
    public static final double kArmKV = 0;
    public static final double kArmKA = 0;


    // Drive Ratio
    public static final double kArmRatio = 1;

    private final TalonFX m_ArmMotor = new TalonFX(kArmId, kArmBus);
    
    private final VelocityVoltage m_ArmOutput = new VelocityVoltage(kArmSpeed);

    public Arm() {
        super();
        // configure Arm
        final TalonFXConfiguration armConfig = new TalonFXConfiguration(); 
        // set contoller gains
        armConfig.Slot0 = new Slot0Configs().withKP(kArmKP).withKI(kArmKI).withKD(kArmKD)
            .withKS(kArmKS).withKV(kArmKV).withKA(kArmKA);
        //invert motor 
        armConfig.MotorOutput.Inverted = kArmInverted; 
        //set ratios 
        armConfig.Feedback.SensorToMechanismRatio = kArmRatio; 
        //set neutral modes 
        m_ArmMotor.setNeutralMode(kArmNeutralMode);
        //Apply Configs 
        m_ArmMotor.getConfigurator().apply(armConfig); 
        
      }

    public void setArmSpeed(double speed) {
    m_ArmOutput.Velocity = speed;
    m_ArmMotor.setControl(m_ArmOutput);
    m_ArmMotor.setNeutralMode(NeutralModeValue.Brake);
    if (speed == 0.0)
      m_ArmMotor.setControl(new StaticBrake());
  }


  /**
   * @brief set the speed of the flywheel
   * 
   * @param speed speed in revolutions per second
   * @return Command
   */
  
  public Command setSpeed(double speed) {
       return this.runOnce(() -> {
         this.setArmSpeed(speed);
       });
     }
   


    // public Command reverse() {
    //         return this.runOnce(() -> {
    //         this.setArmSpeed(-kArmSpeed);
    //         });
    // }

  /**
   * @brief Spin up the flywheel motors
   * 
   * @return Command
   */
  public Command forwards() {
    System.out.println("Test Arm forward");
    return this.runOnce(() -> {
      this.setArmSpeed(kArmSpeed);
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
