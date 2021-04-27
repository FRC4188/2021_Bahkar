// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class Reverse extends CommandBase {

  private Drivetrain drivetrain;
  private double distance;
  private double startX;
  private double startY;

  /** Creates a new Reverse. */
  public Reverse(Drivetrain drivetrain, double distance) { 
    // Use addRequirements() here to declare subsystem dependencies.
 addRequirements(drivetrain); 

 this.drivetrain = drivetrain;
 this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startX = drivetrain.getPose().getX();
    startY = drivetrain.getPose().getY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(-0.7, 0.0, 0.0, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0.0, 0.0, 0.0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.hypot(drivetrain.getPose().getX() - startX, drivetrain.getPose().getY() - startY) >= distance;
  }
}
