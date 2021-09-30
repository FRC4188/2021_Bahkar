// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterVelocity extends InstantCommand {

  Shooter shooter = Shooter.getInstance();
  double velocity;

  public ShooterVelocity(double velocity) {
    addRequirements(shooter);
    this.velocity = velocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setVelocity(velocity);
  }
}
