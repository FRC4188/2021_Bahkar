/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory {
  SwerveControllerCommand controller;
  Drivetrain drivetrain;
  boolean solus;

  ProfiledPIDController thetaController = new ProfiledPIDController(0.2, 0.0, 0.001, new Constraints(Constants.DRIVE_MAX_RADIANS, 2.0 * Constants.DRIVE_MAX_RADIANS));
  PIDController xController = new PIDController(0.5, 0.0, 0.0);
  PIDController yController = new PIDController(0.5, 0.0, 0.0);

  /**
   * Creates a new FollowTrajectory.
   */
  public FollowTrajectory(Drivetrain drivetrain, Trajectory trajectory, boolean solus) {
    this.drivetrain = drivetrain;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    controller = new SwerveControllerCommand(
      trajectory, drivetrain::getPose, drivetrain.getKinematics(), xController, yController, thetaController, drivetrain::setModuleStates, drivetrain);
    drivetrain.resetOdometry(trajectory.getInitialPose());
  }

  public Command getCommand() {
    return (solus) ? controller.andThen(() -> drivetrain.drive(0,0,0, true)) : controller;
  }
}