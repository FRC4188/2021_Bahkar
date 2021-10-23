// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.Constants;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.sensors.Sensors;

public class AutoMagazine extends CommandBase {

  private Hopper hopper = Hopper.getInstance();
  private Shooter shooter = Shooter.getInstance();
  private Turret turret = Turret.getInstance();

  private boolean cont = true;

  private double rpm = 0.0;
  //private int shots = 0;
  //private int count = 0;
  //private boolean counting = false;

  /** Creates a new AutoMagazine. */
  public AutoMagazine(boolean cont) {
    addRequirements(hopper);

    this.cont = cont;
  }

  public AutoMagazine(double rpm, boolean cont) {
    this(cont);

    this.rpm = rpm;
  }

  /*public AutoMagazine(int shots) {
    this.shots = shots;

    counting = true;
  }

  public AutoMagazine(double rpm, int shots) {
    this.rpm = rpm;
    this.shots = shots;

    counting = true;
  }*/

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private boolean lastBeam = true;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean ready = false;

    if (rpm == 0.0) {
      ready = Math.abs(shooter.getVelocity() - Sensors.getInstance().getFormulaRPM()) < Constants.shooter.SHOOTING_TOLERANCE && turret.isAimed();
    } else {
      ready = Math.abs(shooter.getVelocity() - rpm) < Constants.shooter.SHOOTING_TOLERANCE && turret.isAimed();
    }

    if (ready) {
      hopper.set(1.0);
    } else {
      hopper.set(0.0);
    }

    //if (lastBeam && !hopper.getTopBeam()) count++;
    //lastBeam = hopper.getTopBeam();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hopper.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //if (counting) return count == shots;
    //else 
    return cont;
  }
}
