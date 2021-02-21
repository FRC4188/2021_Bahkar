/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  CANSparkMax intakeMotor = new CANSparkMax(41, MotorType.kBrushless);
  CANEncoder intakeEncoder = intakeMotor.getEncoder();
  Solenoid piston = new Solenoid(0);

  /**
   * Creates a new Intake.
   */
  public Intake() {
    resetEncoders();
    motorInits();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void motorInits() {
    intakeMotor.setInverted(true);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    intakeMotor.setOpenLoopRampRate(Constants.Intake.RAMP_RATE);
  }

  /**
   * Sets the power for the intake motor.
   * @param power power input in a range of [-1.0, 1.0]
   */
  public void set(double power) {
    intakeMotor.set(power);
  }

  /**
  * Resets turret encoder value to 0.
  */
  public void resetEncoders() {
    intakeEncoder.setPosition(0.0);
  }

  /**
   * Raises the intake.
   */
  public void raise() {
    piston.set(false);
  }

  /**
   * Lowers the intake.
   */
  public void lower() {
    piston.set(false);
  }

  /**
   * Raises/Lowers the intake to wherever it isn't.
   */
  public void toggle() {
    piston.set(!piston.get());
  }

  /**
   * Method to determine the current position of the intake.
   * @return True if lowered, false if raised.
   */
  public boolean getIsLowered() {
    return piston.get();
  }

  /**
   * Returns the temperature of the intake motor.
   * @return Motor temperature of the intake motor in celsius.
   */
  public double getTemp() {
    return intakeMotor.getMotorTemperature();
  }
}
