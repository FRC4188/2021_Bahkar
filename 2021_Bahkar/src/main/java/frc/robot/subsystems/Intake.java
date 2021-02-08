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
  CANSparkMax intakeMotor = new CANSparkMax(21, MotorType.kBrushless);
  CANEncoder intakeMotorEncoder = intakeMotor.getEncoder();
  Solenoid piston = new Solenoid(0);

  /**
   * Creates a new Intake.
   */
  public Intake() {
    intakeMotor.setInverted(true);
    intakeMotor.setIdleMode(IdleMode.kCoast);
    resetEncoders();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetEncoders() {
    intakeMotorEncoder.setPosition(0);
  }

  public void set(double power) {
    intakeMotor.set(power);
  }

  public double getIntakeMotorTemp() {
    return intakeMotor.getMotorTemperature();
  }

  public double getIntakeMotorPosition() {
    return intakeMotorEncoder.getPosition();
  }

  public void setIntakeRampRate() {
    intakeMotor.setOpenLoopRampRate(Constants.Intake.RAMP_RATE);
  }

  public void raise() {
    piston.set(false);
  }

  public void lower() {
    piston.set(false);
  }

  public void toggle() {
    piston.set(!piston.get());
  }

  public boolean getIsLowered() {
    return piston.get();
  }
}
