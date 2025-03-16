package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Climb extends SubsystemBase {

    public static final String kArmBus = "drivecan";
    public static final int kArmId = 20;
    public static final double kArmPose = 0;
    public static final NeutralModeValue kArmNeutralMode = NeutralModeValue.Brake;
    public static final InvertedValue kArmInverted = InvertedValue.Clockwise_Positive;
    
    //arm controller gains
    public static final double kArmKP = 5; //70
    public static final double kArmKI = 0;
    public static final double kArmKD = 0; //4

    //arm controller feedforward gains
    public static final double kArmKG = 0;
    public static final double kArmKS = 0;
    public static final double kArmKV = 0;
    public static final double kArmKA = 0;
    
    public static final double kCurrentLimit = 40;
      
    // Drive Ratio
    public static final double kArmRatio = 75;

    private final TalonFX m_ArmMotor = new TalonFX(kArmId, kArmBus);
    
    // private final VelocityTorqueCurrentFOC m_ArmOutput = 
    

    public Climb(){
        // configure Arm
        final TalonFXConfiguration armConfig = new TalonFXConfiguration(); 

        // set contoller gains
        armConfig.Slot0 = new Slot0Configs().withKP(kArmKP).withKI(kArmKI).withKD(kArmKD)
            .withKS(kArmKS).withKV(kArmKV).withKA(kArmKA).withKG(kArmKG).withGravityType(GravityTypeValue.Arm_Cosine);
        //invert motor 
        armConfig.MotorOutput.Inverted = kArmInverted; 

        //set ratios 
        armConfig.Feedback.SensorToMechanismRatio = kArmRatio; 
        //set neutral modes 
        m_ArmMotor.setNeutralMode(kArmNeutralMode);

        // set current limit
        armConfig.CurrentLimits.StatorCurrentLimit = kCurrentLimit;
        armConfig.CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
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
 

      public Command climbCommand(){
        return new SequentialCommandGroup(

        this.setVoltage(2),
        new WaitCommand(3),
        this.setVoltage(0)
            
        );
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
        // builder.addDoubleProperty("Target Position", () -> m_ArmOutput.Position,
        // (double target) -> this.goToAngle(target).schedule());
      }
    
    
    
}
