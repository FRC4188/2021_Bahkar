// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hopper;

public class AutoMagazine extends CommandBase {

  private Hopper hopper;
  private Shooter shooter;
  private Turret turret;
  private boolean cont;

  /** Creates a new AutoMagazine. */
  public AutoMagazine(Hopper hopper, Shooter shooter, Turret turret, boolean cont) {
    addRequirements(hopper);

    this.hopper = hopper;
    this.shooter = shooter;
    this.turret = turret;
    this.cont = cont;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean ready = Math.abs(shooter.getLowerVelocity() - 4000.0) < 10.0 && turret.isAimed();

    if (ready) hopper.set(0.8);
    else {
      if (hopper.getBeam()) hopper.set(0.0);
      else hopper.set(0.35);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !cont;
  }
}
