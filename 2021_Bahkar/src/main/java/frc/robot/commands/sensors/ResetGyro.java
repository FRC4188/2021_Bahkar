// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sensors;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.drive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Sensors;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetGyro extends InstantCommand {

  Sensors sensors;
  Drivetrain drivetrain;

  public ResetGyro(Sensors sensors, Drivetrain drivetrain) {
    this.sensors = sensors;
    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensors.resetGyro();
    drivetrain.resetOdometry(new Pose2d(drivetrain.getPose().getTranslation(), new Rotation2d()));
  }
}
