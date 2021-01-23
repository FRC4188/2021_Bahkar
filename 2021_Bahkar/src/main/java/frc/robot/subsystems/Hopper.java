/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

  private final TalonFX hopperMotor = new TalonFX(9);
  private Sensors sensors;

  /**
   * Creates a new Hopper.
   */
  public Hopper(Sensors sensors) {
    this.sensors = sensors;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double percentage) {
    hopperMotor.set(ControlMode.PercentOutput, percentage);
  }

  public boolean getBeam() {
    return (sensors.getTopBeam());
  }
}
