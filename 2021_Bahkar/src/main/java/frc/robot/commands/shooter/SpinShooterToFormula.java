/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Sensors;

public class SpinShooterToFormula extends CommandBase {

  private Shooter shooter;
  private Sensors sensors;
  private Hood hood;

  /**
   * Creates a new SpinShooterToFormula.
   */
  public SpinShooterToFormula(Shooter shooter, Sensors sensors, Hood hood) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, sensors, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setVelocity(sensors.formulaRPMandAngle()[0]);
    hood.setHoodAngle(sensors.formulaRPMandAngle()[1]);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setPercentage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  
}
