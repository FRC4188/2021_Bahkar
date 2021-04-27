/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.components.LEDPanel;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  private LEDPanel ledPanel;

  private PowerDistributionPanel pdp = new PowerDistributionPanel();
  private Notifier shuffle = new Notifier(() -> updateShuffle());

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    ledPanel = null;//new LEDPanel(10);

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(ledPanel);
    //shuffle.startPeriodic(0.1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    //tempManager.run();

    /*ledPanel.set(LEDPanel.SYSTEM.GENERAL, 0, RobotController.getBatteryVoltage() > 12.0 ? LEDPanel.BEHAVIOR.OFF :
                                             RobotController.getBatteryVoltage() > 10.0 ? LEDPanel.BEHAVIOR.BLINK :
                                                                                          LEDPanel.BEHAVIOR.ON);*/

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    //m_robotContainer.resetRobot();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.startTele();

    if (RobotController.getBatteryVoltage() < 12.7) DriverStation.reportWarning("Battery voltage too low; please change battery.", false);
    //m_robotContainer.resetRobot();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //tempManager.run();
    //bop.run();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  private void updateShuffle() {
    SmartDashboard.putNumber("Channel 1", pdp.getCurrent(0));
    SmartDashboard.putNumber("Channel 2", pdp.getCurrent(1));
    SmartDashboard.putNumber("Channel 3", pdp.getCurrent(2));
    SmartDashboard.putNumber("Channel 4", pdp.getCurrent(3));
    SmartDashboard.putNumber("Channel 5", pdp.getCurrent(4));
    SmartDashboard.putNumber("Channel 6", pdp.getCurrent(5));
    SmartDashboard.putNumber("Channel 7", pdp.getCurrent(6));
    SmartDashboard.putNumber("Channel 8", pdp.getCurrent(7));
    SmartDashboard.putNumber("Channel 9", pdp.getCurrent(8));
    SmartDashboard.putNumber("Channel 10", pdp.getCurrent(9));
    SmartDashboard.putNumber("Channel 11", pdp.getCurrent(10));
    SmartDashboard.putNumber("Channel 12", pdp.getCurrent(11));
    SmartDashboard.putNumber("Channel 13", pdp.getCurrent(12));
    SmartDashboard.putNumber("Channel 14", pdp.getCurrent(13));
    SmartDashboard.putNumber("Channel 15", pdp.getCurrent(14));
    SmartDashboard.putNumber("Channel 16", pdp.getCurrent(15));

  }
}
