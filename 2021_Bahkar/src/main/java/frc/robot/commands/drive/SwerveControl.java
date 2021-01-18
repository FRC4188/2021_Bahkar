/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class SwerveControl extends CommandBase {
  Drivetrain drivetrain;
  Trajectory trajectory;
  double theta;

  Timer timer = new Timer();
  ProfiledPIDController thetaController = new ProfiledPIDController(0.2, 0.0, 0.0, new Constraints(Constants.DRIVE_MAX_RADIANS, 2.0 * Constants.DRIVE_MAX_RADIANS));
  PIDController xController = new PIDController(1.5, 0.0, 0.0);
  PIDController yController = new PIDController(1.5, 0.0, 0.0
  );
  /**
   * Creates a new SwerveControl.
   */
  public SwerveControl(Drivetrain drivetrain, Trajectory trajectory) {
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetOdometry(trajectory.getInitialPose());
    theta = drivetrain.getPose().getRotation().getDegrees();

    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d setPoint = trajectory.sample(timer.get()).poseMeters;
    Pose2d c_pose = drivetrain.getPose();

    double xVel = xController.calculate(c_pose.getTranslation().getX(), setPoint.getTranslation().getX()) * Constants.DRIVE_MAX_VELOCITY;
    double yVel = yController.calculate(c_pose.getTranslation().getY(), setPoint.getTranslation().getY()) * Constants.DRIVE_MAX_VELOCITY;
    double thetaVel = thetaController.calculate(c_pose.getRotation().getDegrees(), theta);

    drivetrain.drive(xVel, yVel, thetaVel, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setChassisSpeeds(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() == trajectory.getTotalTimeSeconds();
  }
}
