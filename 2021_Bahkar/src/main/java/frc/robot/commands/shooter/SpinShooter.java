/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class SpinShooter extends CommandBase {
  private Shooter shooter;
  private double velocity;

  private boolean cont;
  /**
   * Creates a new SpinShooter.
   */
  public SpinShooter(Shooter shooter, double velocity, boolean cont) {
    addRequirements(shooter);
    this.shooter = shooter;
    this.velocity = velocity;
    this.cont = cont;
  }

  /**
   * Creates a new SpinShooter which will run until interrupted.
   */
  public SpinShooter(Shooter shooter, double velocity) {
    this(shooter, velocity, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocity(velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setVelocity(Constants.shooter.IDLE_VEL);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !cont;
  }
}
