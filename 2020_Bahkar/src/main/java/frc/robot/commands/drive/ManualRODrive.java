/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.utils.CspController;

public class ManualRODrive extends CommandBase {

  private SwerveDrive drivetrain;
  private CspController input;

  /**
   * Creates a new ManualDrive.
   */
  public ManualRODrive(SwerveDrive drivetrain, CspController input) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.input = input;
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(input.getX(Hand.kRight), input.getY(Hand.kRight), input.getX(Hand.kLeft));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
