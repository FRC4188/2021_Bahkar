// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.DSolenoid;

public class Hood extends SubsystemBase {

  private static Hood instance;

  public static synchronized Hood getInstance() {
    if (instance == null) instance = new Hood();
    return instance;
  }

  private DSolenoid piston = new DSolenoid(6, 5);

  private Notifier shuffle = new Notifier(() -> shuffle());

  /** Creates a new Hood. */
  public Hood() {
    CommandScheduler.getInstance().registerSubsystem(this);
    shuffle.startPeriodic(0.2);
  }

  @Override
  public void periodic() {}

  private void shuffle() {
    SmartDashboard.putBoolean("Hood Solenoid", getRaised());
  }

  public void setPosition(boolean position) {
    piston.set(position);
    ;
  }

  public boolean getRaised() {
    return piston.get();
  }

  public void relax() {
    piston.relax();
  }
}
