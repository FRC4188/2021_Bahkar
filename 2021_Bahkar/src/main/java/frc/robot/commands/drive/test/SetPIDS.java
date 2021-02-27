// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;

public class SetPIDS extends InstantCommand {
  Drivetrain drivetrain;
  /** Creates a new SetPIDS. */
  public SetPIDS(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.setAnglePID(
      SmartDashboard.getNumber("Set Angle kP", 0.0),
      0.0,
      0.0
    );
    drivetrain.setSpeedPID(
      SmartDashboard.getNumber("Set Speed kP", 0.0),
      0.0,
      0.0);
  }
}
