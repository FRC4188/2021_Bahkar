/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  CANSparkMax intakeMotor = new CANSparkMax(21, MotorType.kBrushless);
  Solenoid piston = new Solenoid(0);
  /**
   * Creates a new Intake.
   */
  public Intake() {
    intakeMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double power) {
    intakeMotor.set(power);
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

  public boolean getLowered() {
    return piston.get();
  }
}
