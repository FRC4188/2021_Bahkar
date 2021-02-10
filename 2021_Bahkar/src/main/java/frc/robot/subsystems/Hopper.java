/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.components.CSPFalcon;

public class Hopper extends SubsystemBase {

  private final CSPFalcon hopperMotor = new CSPFalcon(9);
  private Sensors sensors;

  /**
   * Creates a new Hopper.
   */
  public Hopper(Sensors sensors) {
    this.sensors = sensors;
    hopperMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the power to the hopper motor.
   * @param percentage power to set the motor to in a range of [-1.0, 1.0].
   */
  public void set(double percentage) {
    hopperMotor.set(percentage);
  }

  /**
   * Gets the state of the beam breaker.
   * @return True if the beam is unbroken, false if the beam is broken.
   */
  public boolean getBeam() {
    return (sensors.getTopBeam());
  }

  /**
   * Returns the temperature of the hopper motor.
   * @return Temperature of hopper in celsius.
   */
  public double getTemp() {
    return hopperMotor.getTemperature();
  }
}
