// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Sensors;

public class AutoMagQuantity extends CommandBase {

  private Hopper hopper;
  private Shooter shooter;
  private Turret turret;
  private Sensors sensors;

  private double RPM_GOAL = 4000.0;

  private int quantity;
  private boolean lastTop = true;

  /** Creates a new AutoMagazine. */
  public AutoMagQuantity(Hopper hopper, Shooter shooter, Turret turret, Sensors sensors, int quantity) {
    addRequirements(hopper);

    this.hopper = hopper;
    this.shooter = shooter;
    this.turret = turret;
    this.sensors = sensors;
    this.quantity = quantity;
  }

  public AutoMagQuantity(Hopper hopper, Shooter shooter, Turret turret, Sensors sensors, int quantity, double rpm) {
    this(hopper, shooter, turret, sensors, quantity);
    RPM_GOAL = rpm;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean ready = Math.abs(shooter.getLowerVelocity() - RPM_GOAL) < 50.0 && turret.isAimed();
    boolean top = sensors.getTopBeam();

    System.out.println(String.valueOf(ready));

    if (ready) {
      hopper.set(1.0);
    } else {
      hopper.set(0.0);
    }

    if (!lastTop && top) quantity--;

    lastTop = top;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return quantity == 0;
  }
}
