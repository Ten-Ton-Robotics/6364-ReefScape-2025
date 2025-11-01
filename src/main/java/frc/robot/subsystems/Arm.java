package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    public static final String ARM_BUS = "rio";
    public static final int ARM_ID = 3;
    public static final double ARM_POSE = 0;
    public static final NeutralModeValue ARM_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final InvertedValue ARM_INVERTED = InvertedValue.Clockwise_Positive;
    
    //arm controller gains
    public static final double ARM_KP = 550; //90
    public static final double ARM_KI = 0;
    public static final double ARM_KD = 25.5; //8

    //arm controller feedforward gains
    public static final double ARM_KG = 4;
    public static final double ARM_KS = 0;
    public static final double ARM_KV = 0;
    public static final double ARM_KA = 0;

    // Motion Magic Consts
    public static final double CRUISE_VELOCITY = 0.6; // 0.5s
    public static final double ACCELERATION = 1.2;
    public static final double JERK = 1.8;


    public static final double CURRENT_LIMIT = 12;

    // Arm Pose
    public static final double MAX_POSITION = 0.26220703125;

    // Drive Ratio
    public static final double ARM_RATIO = 75;

    private final TalonFX m_ArmMotor = new TalonFX(ARM_ID, ARM_BUS);
    
    private final MotionMagicTorqueCurrentFOC m_ArmOutput = new MotionMagicTorqueCurrentFOC(ARM_POSE);

    public Arm() {
      
        super();
        // configure Arm
        final TalonFXConfiguration armConfig = new TalonFXConfiguration(); 

        // set contoller gains
        armConfig.Slot0 = new Slot0Configs().withKP(ARM_KP).withKI(ARM_KI).withKD(ARM_KD)
            .withKS(ARM_KS).withKV(ARM_KV).withKA(ARM_KA).withKG(ARM_KG).withGravityType(GravityTypeValue.Arm_Cosine);
        
        //invert motor 
        armConfig.MotorOutput.Inverted = ARM_INVERTED; 

        //set ratios 
        armConfig.Feedback.SensorToMechanismRatio = ARM_RATIO; 
        
        //set neutral modes 
        m_ArmMotor.setNeutralMode(ARM_NEUTRAL_MODE);

        // set current limit
        armConfig.CurrentLimits.StatorCurrentLimit = CURRENT_LIMIT;
        armConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIMIT;
        armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Configure Motion Magic settings
        armConfig.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY;
        armConfig.MotionMagic.MotionMagicAcceleration = ACCELERATION;
        armConfig.MotionMagic.MotionMagicJerk = JERK;

        //Apply Configs 
        m_ArmMotor.getConfigurator().apply(armConfig); 
        m_ArmMotor.setPosition(MAX_POSITION);
      }
    
  //   public void setArmSpeed(double speed) {
  //   m_ArmOutput.Velocity = speed;
  //   m_ArmMotor.setControl(m_ArmOutput);
  //   m_ArmMotor.setNeutralMode(NeutralModeValue.Brake);
  //   if (speed == 0.0)
  //     m_ArmMotor.setControl(new StaticBrake());
  // }


  /**
   * @brief set the speed of the arm motor
   * 
   * @param speed speed in revolutions per second
   * @return Command
   */

  // public Command setSpeed(double speed) {
  //      return this.runOnce(() -> {
  //        this.setArmSpeed(speed);
  //      });
  //    }
   


  //   public Command reverse() {
  //           return this.runOnce(() -> {
  //           this.setArmSpeed(-kArmSpeed);
  //           });
  //   }

  // /**
  //  * @brief Spin up the flywheel motors
  //  * 
  //  * @return Command
  //  */
  // public Command forwards() {
  //   System.out.println("Test Arm forward");
  //   return this.runOnce(() -> {
  //     this.setArmSpeed(kArmSpeed);
  //   });
  // }
  

  /**
   * @brief Stop the Arm motors
   * 
   * @return Command
   */
  public Command stop() {
    return new InstantCommand( () -> m_ArmMotor.setControl(new StaticBrake()));
  }

/**
 *   @brief 
 * 
 *   @param Set the speed of the Arm in rotations per
 *   @return command
  */
  public Command goToAngle(double position) {
    return this.runOnce(() -> {
      System.out.println("Setting New go to angle!!!!");
      m_ArmOutput.Position = position;
      m_ArmMotor.setControl(m_ArmOutput);
    
    });
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
    super.initSendable(builder);
    
    builder.addDoubleProperty("Position", () -> m_ArmMotor.getPosition().getValueAsDouble(),
    (double position) -> m_ArmMotor.setPosition(position));
    builder.addDoubleProperty("Target Position", () -> m_ArmOutput.Position,
    (double target) -> this.goToAngle(target).schedule());

    builder.addDoubleProperty("Voltage", () -> m_ArmMotor.getMotorVoltage().getValueAsDouble(),
    null);

    builder.addDoubleProperty("Supply Current", () -> m_ArmMotor.getSupplyCurrent().getValueAsDouble(),
    null);

    builder.addDoubleProperty("Stator Current", () -> m_ArmMotor.getStatorCurrent().getValueAsDouble(),
    null);



    
// call the superclass method
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
