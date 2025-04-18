// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private boolean climbrunce = false;

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_robotContainer.m_drivetrain.getOdometryThread().start();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.updatePoseEstimator();

    // if(m_robotContainer.m_koral_sensor)
    
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
  public void autonomousPeriodic() {

    m_robotContainer.updatePoseEstimator();

    // if(!m_robotContainer.m_koral_sensor.get()){
    //   m_robotContainer.m_Intake.koralControlCommand(0.075).schedule();
    // } else{
    //   m_robotContainer.m_Intake.forwards(true).;
    // }

  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.objectDetected.onFalse(m_robotContainer.m_Intake.koralControlCommand(0.075)); //-0.38
    m_robotContainer.objectDetected.onTrue(m_robotContainer.m_Intake.forwards(true));


    // m_robotContainer.objectDetected.onTrue(m_robotContainer.m_Intake.koralControlCommand(0.075)); //-0.38
    // m_robotContainer.objectDetected.onFalse(m_robotContainer.m_Intake.forwards(true));


    m_robotContainer.m_Arm.goToAngle(0.26).schedule();

    if (m_robotContainer.m_koral_sensor.get()) {
      m_robotContainer.m_Intake.forwards(true).alongWith(m_robotContainer.m_Arm.goToAngle(0.26)).schedule();
    }

    // m_robotContainer.m_drivetrain.runOnce(() -> m_robotContainer.m_drivetrain.seedFieldCentric());

  }

  @Override
  public void teleopPeriodic() {
    // if(climbrunce == false){
    //   climbrunce = true;
    //   m_robotContainer.m_climber.goToPosition(75).schedule();
    // }
  }

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
  public void simulationPeriodic() {}
}
