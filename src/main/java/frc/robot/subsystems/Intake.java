package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import edu.wpi.first.networktables.BooleanPublisher;
// import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;

public class Intake extends SubsystemBase {
    
    // (Did you mean Donner kebab?) 
    public static final String kUpperMotorBus = "rio";
    public static final String kLowerMotorBus = "rio";

    public static final int kUpperMotorId = 1;
    public static final int kLowerMotorId = 2;

    public static final double kUpperSpeed = 50;
    public static final double kLowerSpeed = 80;

    public static final NeutralModeValue kUpperNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue kLowerNeutralMode = NeutralModeValue.Brake;

    public static final InvertedValue kUpperMotorInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kLowerMotorInverted = InvertedValue.CounterClockwise_Positive;
    
    // public final DigitalInput m_koral_sensor = new DigitalInput(0);

    // private boolean m_isWaiting = false;
    // private Timer m_timer = new Timer();

    // Upper motor controller gains
    public static final double kUpperKP = 0.35;
    public static final double kUpperKI = 0;
    public static final double kUpperKD = 0;

    // Upper motor controller feedforward gains
    public static final double kUpperKS = 0;
    public static final double kUpperKV = 0;
    public static final double kUpperKA = 0;

    // Lower motor controller gains
    public static final double kLowerKP = 0.35;
    public static final double kLowerKI = 0;
    public static final double kLowerKD = 0;

    // Lower motor controller feedforward k
    public static final double kLowerKS = 0;
    public static final double kLowerKV = 0;
    public static final double kLowerKA = 0;

    // Drive Ratio
    public static final double kUpperRatio = 1;
    public static final double kLowerRatio = 1;

    private final TalonFX m_upperMotor = new TalonFX(kUpperMotorId, kUpperMotorBus);
    private final TalonFX m_lowerMotor = new TalonFX(kLowerMotorId, kLowerMotorBus);
    
    // Motor Outputs
    private final VelocityTorqueCurrentFOC m_upperOutput = new VelocityTorqueCurrentFOC(kUpperSpeed);
    private final VelocityTorqueCurrentFOC m_lowerOutput = new VelocityTorqueCurrentFOC(kLowerSpeed);

    private boolean on = false;
    public boolean sensor_out;
    
    private double kCurrentLimit = 10;
      
    public Intake() {
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
        m_upperMotor.getConfigurator().apply(upperConfig);
        m_lowerMotor.getConfigurator().apply(lowerConfig);

        upperConfig.CurrentLimits.StatorCurrentLimit = kCurrentLimit;
        lowerConfig.CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
        upperConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        lowerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // commands

        // TODO: SET A CURRENT LIMIT AFTER WE GET FOC WORKING

      }

    public void setUpperSpeed(double speed) {
    m_upperOutput.Velocity = speed;
    m_upperMotor.setControl(m_upperOutput);
    m_upperMotor.setNeutralMode(NeutralModeValue.Brake);
    if (speed == 0.0)
      m_upperMotor.setControl(new StaticBrake());
  }


public Command koralControlCommand(double waitseconds) {
  // return this.runOnce( () -> {
  //     new WaitCommand(waitseconds);
  //     this.stop();

  // });
  return new ParallelCommandGroup(
     new SequentialCommandGroup(
        new WaitCommand(waitseconds),
        this.stop()

    )
  );

  // return Commands.none();
}


@Override
public void periodic() {
    // sensor_out = !m_koral_sensor.get(); // Poll the sensor  
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
   * @brief set the speed of the intake motors
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
  
     /**
   * @brief reverse the speed of the intake motors
   * 
   * @param speed speed in revolutions per second
   * @return Command
   */
  public Command reverse(double speed) {
            return this.runOnce(() -> {
            this.setUpperSpeed(-speed);
            this.setLowerSpeed(-speed);
            });
    }


    public Command backup1andahalfinch(){
      return new SequentialCommandGroup(

        reverse(25),
        new WaitCommand(0.075),
        stop()
        
      );
    }


    public Command backup3inch(){
      return new SequentialCommandGroup(

        reverse(25),
        new WaitCommand(0.15),
        stop()
        
      );
    }

    public Command forward3inch(){
      return new SequentialCommandGroup(

        setSpeed(25),
        new WaitCommand(0.075),
        stop()
        
      );
    }


/**
   * Spin up the intake motors in the same direction 
   * 
   * 
   * @return Command
   */
  public Command forwardsame(){
    return this.runOnce(() -> {
      
        this.setUpperSpeed(-50);
        this.setLowerSpeed(50);
    });
  }

  public Command reversesame(){
    return this.runOnce(() -> {
        this.setUpperSpeed(100);
        this.setLowerSpeed(-100);
    });
  }

  /**
   * @brief Spin up the intake motors in opposite directions 
   * 
   * @return Command
   */
  public Command forwards(boolean load) {
    return this.runOnce(() -> {
      
      if(load){
        this.setUpperSpeed(kUpperSpeed);
        this.setLowerSpeed(kLowerSpeed);

      }else{
        this.setUpperSpeed(100);
        this.setLowerSpeed(100);

      }
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

    builder.addBooleanProperty("Koral detected", () -> !RobotContainer.m_koral_sensor.get(), null);
    builder.addBooleanProperty("Intake On", () -> on, null);
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
