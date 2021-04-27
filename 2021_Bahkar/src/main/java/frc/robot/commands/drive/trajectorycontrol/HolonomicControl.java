/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.trajectorycontrol;

import edu.wpi.first.wpilibj.controller.HolonomicDriveController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.trajectory.Waypoints;

public class HolonomicControl extends CommandBase {
  Drivetrain drivetrain;
  Trajectory trajectory;
  Rotation2d angle;
  int index = 0;

  Timer timer = new Timer();

  ProfiledPIDController thetaController = new ProfiledPIDController(0.7, 0.0, 0.12, new Constraints(Constants.drive.MAX_RADIANS, 2.0 * Constants.drive.MAX_RADIANS));
  PIDController xController = new PIDController(5.2, 0.0, 0.01);
  PIDController yController = new PIDController(5.2, 0.0, 0.01);

  HolonomicDriveController controller = new HolonomicDriveController(xController, yController, thetaController);

  /**
   * Creates a new SwerveControl.
   */
  public HolonomicControl(Drivetrain drivetrain, Trajectory trajectory, Rotation2d angle) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
    this.trajectory = trajectory;
    this.angle = angle;

    controller.setEnabled(true);
  }

  public HolonomicControl(Drivetrain drivetrain, Waypoints waypoints, Rotation2d rotation) {
    this(drivetrain, TrajectoryGenerator.generateTrajectory(waypoints.getPoses(), waypoints.getConfig()), rotation);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = drivetrain.getPose();
    ChassisSpeeds calculated = controller.calculate(new Pose2d(pose.getX(), -pose.getY(), pose.getRotation()), trajectory.sample(timer.get()), angle);
    drivetrain.drive(calculated.vxMetersPerSecond / Constants.drive.MAX_VELOCITY, calculated.vyMetersPerSecond / Constants.drive.MAX_VELOCITY, thetaController.calculate(pose.getRotation().getRadians(), angle.getRadians()), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setChassisSpeeds(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Pose2d endPose = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;

    return timer.get() >= trajectory.getTotalTimeSeconds();/* && 
    Math.abs(drivetrain.getPose().getX() - endPose.getX()) < 0.2 &&
    Math.abs(drivetrain.getPose().getY() - endPose.getY()) < 0.2;*/
  }
}
