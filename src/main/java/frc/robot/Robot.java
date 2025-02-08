// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX; 
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units; 






public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  //TODO Get proper values I do not trust CHATGPT
  private static final double MOTOR_KV = 5330; // RPM/V (example)
  private static final double MOTOR_KT = 0.000115; // Nm/A (example)
  static LinearSystem<N2, N1, N2> motorSystem = LinearSystemId.createDCMotorSystem(MOTOR_KV,MOTOR_KT);

    
  private final RobotContainer m_robotContainer;
  private static double m_GearRatio = 10.0;
  private final DCMotorSim m_motorSimModel = new DCMotorSim(motorSystem, DCMotor.getFalcon500(0), 0.003); 

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.m_drivetrain.getOdometryThread().start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {
    TalonFX m_talonFX = new TalonFX(0);

    TalonFXSimState TalonFXSim = m_talonFX.getSimState();

    TalonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
    
    var motorVoltage = TalonFXSim.getMotorVoltage(); 
    m_motorSimModel.setInputVoltage(motorVoltage);
    m_motorSimModel.update(0.020);

   TalonFXSim.setRawRotorPosition(
    m_GearRatio * m_motorSimModel.getAngularPositionRotations()
   );
   TalonFXSim.setRotorVelocity(
    m_GearRatio * Units.radiansToRotations(m_motorSimModel.getAngularVelocityRadPerSec())
   );

  }
}
