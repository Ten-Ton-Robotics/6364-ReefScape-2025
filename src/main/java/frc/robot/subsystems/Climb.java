package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

    public static final String ARM_BUS = "drivecan";
    public static final int ARM_ID = 20;
    public static final double ARM_POSE = 0;
    public static final NeutralModeValue ARM_NEUTRAL_MODE = NeutralModeValue.Brake;
    public static final InvertedValue ARM_INVERTED = InvertedValue.Clockwise_Positive;
    
    //arm controller gains
    public static final double ARM_KP = 5; //70
    public static final double ARM_KI = 0;
    public static final double ARM_KD = 0; //4

    //arm controller feedforward gains
    public static final double ARM_KG = 0;
    public static final double ARM_KS = 0;
    public static final double ARM_KV = 0;
    public static final double ARM_KA = 0;
    
    public static final double CURRENT_LIST = 40;
    
    // Drive Ratio
    public static final double ARM_RATIO = 1;
    private final TalonFX m_ArmMotor = new TalonFX(ARM_ID, ARM_BUS);
    private final PositionVoltage m_ArmOutput = new PositionVoltage(ARM_POSE);
    private final VelocityVoltage m_VelocityOutput = new VelocityVoltage(ARM_POSE);

    public Climb(){
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
        armConfig.CurrentLimits.StatorCurrentLimit = CURRENT_LIST;
        armConfig.CurrentLimits.SupplyCurrentLimit = CURRENT_LIST;
        armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    
        //Apply Configs 
        m_ArmMotor.getConfigurator().apply(armConfig); 
        m_ArmMotor.setPosition(0);
        
    }

      public void setClimbVoltage(double voltage) {
        // m_ArmOutput.Velocity = speed;
        m_ArmMotor.setControl(new VoltageOut(voltage));
        m_ArmMotor.setNeutralMode(NeutralModeValue.Brake);
        if (voltage == 0.0)
          m_ArmMotor.setControl(new StaticBrake());
      }

      public Command setVoltage(double voltage) {
        return this.runOnce(() -> {
          this.setClimbVoltage(voltage);
        });
      }

      public Command goToPosition(double position){
        return this.runOnce(() -> {
        m_ArmOutput.Position = position;
        m_ArmMotor.setControl(m_ArmOutput);
        });
      }

      public Command setVelocity(double velocity){
        return this.runOnce(() ->{
          m_VelocityOutput.Velocity = velocity;
          m_ArmMotor.setControl(m_VelocityOutput);
        });
      }

      public Command stop(){
        return this.runOnce( () ->{

          this.setClimbVoltage(0);


        });
      }

      @Override
      public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        
        builder.addDoubleProperty("Position", () -> m_ArmMotor.getPosition().getValueAsDouble(),
        (double position) -> m_ArmMotor.setPosition(position));

        builder.addDoubleProperty("Motor Voltage", () -> m_ArmMotor.getMotorVoltage().getValueAsDouble(), null);
        builder.addDoubleProperty("Motor Amperage Supply", () -> m_ArmMotor.getSupplyCurrent().getValueAsDouble(), null);

        // builder.addDoubleProperty("Target Position", () -> m_ArmOutput.Position,
        // (double target) -> this.goToAngle(target).schedule());
      }
}
