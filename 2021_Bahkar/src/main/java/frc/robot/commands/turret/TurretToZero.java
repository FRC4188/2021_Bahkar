package frc.robot.commands.turret;

import frc.robot.subsystems.Turret;

public class TurretToZero extends TurretAngle {
  
  Turret turret;
  
  /**
   * Creates a new TurretToZero.
   */
  public TurretToZero(Turret turret) {
    super(turret, 0.0);
  }
}
