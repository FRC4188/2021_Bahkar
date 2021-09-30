// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {

  private static Hopper instance;

  public synchronized static Hopper getInstance() {
    if (instance == null) instance = new Hopper();
    return instance;
  }

  WPI_TalonFX hopperMotor = new WPI_TalonFX(9);

  /** Creates a new Hopper. */
  public Hopper() {
    CommandScheduler.getInstance().registerSubsystem(this);

    hopperMotor.configFactoryDefault();
    hopperMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    hopperMotor.configOpenloopRamp(Constants.hopper.RAMP_RATE);
    hopperMotor.setInverted(true);
  }

  @Override
  public void periodic() {
  }

  public void set(double power) {
    hopperMotor.set(power);
  }

  public double getTemperature() {
    return hopperMotor.getTemperature();
  }
}
