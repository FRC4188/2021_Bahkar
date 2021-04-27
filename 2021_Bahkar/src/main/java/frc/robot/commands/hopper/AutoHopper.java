/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Sensors;

public class AutoHopper extends CommandBase {
  Hopper hopper;
  Sensors sensors;
  boolean cont;
  /**
   * Creates a new AutoHopper.
   */
  public AutoHopper(Hopper hopper, Sensors sensors, boolean cont) {
    addRequirements(hopper);
    this.hopper = hopper;
    this.sensors = sensors;
    this.cont = cont;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!sensors.getTopBeam()) hopper.set(0.0);
    //else if (!sensors.getMidBeam()) hopper.set(0.15);
    else hopper.set(0.2);
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
