/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;

import java.util.Iterator;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.HolonomicDriveController;
import frc.robot.utils.trajectory.CSPSwerveTrajectory;

public class SwerveControl extends CommandBase {
  Drivetrain drivetrain;
  Trajectory trajectory;
  Iterator<Double> times;
  Iterator<Double> angles;
  double theta;

  double nextTime;
  double angle;

  Timer timer = new Timer();

  ProfiledPIDController thetaController = new ProfiledPIDController(0.2, 0.0, 0.0, new Constraints(Constants.Drive.MAX_RADIANS, 2.0 * Constants.Drive.MAX_RADIANS));
  PIDController xController = new PIDController(1.5, 0.0, 0.0);
  PIDController yController = new PIDController(1.5, 0.0, 0.0);

  HolonomicDriveController controller = new HolonomicDriveController(xController, yController, thetaController);

  /**
   * Creates a new SwerveControl.
   */
  public SwerveControl(Drivetrain drivetrain, CSPSwerveTrajectory trajectory) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.trajectory = trajectory.getTrajectory();
    this.times = trajectory.getTimes().iterator();
    this.angles = trajectory.getAngles().iterator();

    controller.setEnabled(true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();

    times.next();
    nextTime = times.next();
    angle = angles.next();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = drivetrain.getPose();
    double time = timer.get();

    if (time >= nextTime) {
      angle = angles.next();
      if (times.hasNext()) nextTime = times.next();
    }

    drivetrain.setChassisSpeeds(controller.calculate(pose, trajectory.sample(time), Rotation2d.fromDegrees(angle)));
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
