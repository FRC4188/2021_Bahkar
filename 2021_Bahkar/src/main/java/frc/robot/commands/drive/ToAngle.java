/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ToAngle extends CommandBase {
  Drivetrain drivetrain;
  double angle;

  ProfiledPIDController thetaController = new ProfiledPIDController(0.25, 0.0, 0.0, new Constraints(Constants.drive.MAX_VELOCITY, Constants.drive.MAX_ACCEL));
  /**
   * Creates a new ToAngle.
   */
  public ToAngle(Drivetrain drivetrain, double angle) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;

    thetaController.setGoal(angle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, thetaController.calculate(-drivetrain.getPose().getRotation().getDegrees()));
    drivetrain.setChassisSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return thetaController.atSetpoint();
  }
}
