/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class ToPosition extends CommandBase {
  Drivetrain drivetrain;

  double x, y, Angle;

  ProfiledPIDController thetaController = new ProfiledPIDController(0.2, 0.0, 0.0, new Constraints(Constants.Drive.MAX_RADIANS, 2.0 * Constants.Drive.MAX_RADIANS));
  ProfiledPIDController xController = new ProfiledPIDController(1.5, 0.0, 0.0, new Constraints(Constants.Drive.Auto.MAX_VELOCITY, Constants.Drive.Auto.MAX_ACCEL));
  ProfiledPIDController yController = new ProfiledPIDController(1.5, 0.0, 0.0, new Constraints(Constants.Drive.Auto.MAX_VELOCITY, Constants.Drive.Auto.MAX_ACCEL));

  /**
   * Creates a new ToPosition.
   */
  public ToPosition(Drivetrain drivetrain, double x, double y) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.x = x;
    this.y = y;

    xController.setGoal(x);
    yController.setGoal(y);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.setGoal(drivetrain.getPose().getRotation().getDegrees());
    drivetrain.resetOdometry(new Pose2d());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = drivetrain.getPose();
    double xVel = xController.calculate(pose.getTranslation().getX()) * Constants.Drive.Auto.MAX_VELOCITY;
    double yVel = -yController.calculate(pose.getTranslation().getY()) * Constants.Drive.Auto.MAX_VELOCITY;
    double thetaVel = thetaController.calculate(pose.getRotation().getDegrees());

    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVel, yVel, thetaVel, pose.getRotation());

    drivetrain.setChassisSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }
}
