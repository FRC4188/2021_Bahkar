// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {

  private static Hood instance;

  public synchronized static Hood getInstance() {
    if (instance == null) instance = new Hood();
    return instance;
  }

  //LinearServos servos = new LinearServos(0, 1);

  private Solenoid piston = new Solenoid(2);

  /** Creates a new Hood. */
  public Hood() {
    SmartDashboard.putNumber("Set Hood Position (mm)", 0.0);
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  @Override
  public void periodic() {
  }

  public void setPosition(boolean position) {
    //servos.set(position);
    piston.set(position);
  }
}
