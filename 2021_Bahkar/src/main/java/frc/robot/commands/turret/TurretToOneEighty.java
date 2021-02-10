package frc.robot.commands.turret;

import frc.robot.subsystems.Turret;

public class TurretToOneEighty extends TurretAngle {
  /**
   * Creates a new TurretToOneEighty.
   */
  public TurretToOneEighty(Turret turret) {
    super(turret, 180.0);
  }
}
