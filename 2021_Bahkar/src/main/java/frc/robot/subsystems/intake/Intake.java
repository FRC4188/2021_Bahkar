// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private static Intake instance;

  public synchronized static Intake getInstace() {
    if (instance == null) instance = new Intake();
    return instance;
  }

  private FourBar fourBar = new FourBar(0);
  private WPI_TalonFX intakeMotor = new WPI_TalonFX(14);

  /** Creates a new Intake. */
  private Intake() {
    CommandScheduler.getInstance().registerSubsystem(this);

    intakeMotor.setNeutralMode(NeutralMode.Brake);
    intakeMotor.configOpenloopRamp(Constants.intake.RAMP_RATE);
    intakeMotor.setInverted(true);
  }

  @Override
  public void periodic() {
  }

  public void set(double power) {
    intakeMotor.set(power);
  }

  public void setRaised(boolean raised) {
    fourBar.setRaised(raised);
  }

  public boolean getRaised() {
    return fourBar.getRaised();
  }

  public void toggleRaised() {
    fourBar.setRaised(!getRaised());
  }

  public double getTemperature() {
    return intakeMotor.getTemperature();
  }
}
