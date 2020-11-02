/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Drivetrain;

public class FollowTrajectory extends CommandBase {
  RamseteController controller = new RamseteController();

  Drivetrain drivetrain;
  Sensors sensors;
  Trajectory trajectory;

  long start;
  double time;
  double totalTime;

  /**
   * Creates a new FollowTrajectory.
   */
  public FollowTrajectory(Drivetrain drivetrain, Sensors sensors, Trajectory trajectory) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.sensors = sensors;
    this.trajectory = trajectory;

    totalTime = trajectory.getTotalTimeSeconds();

    start = System.currentTimeMillis();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    time = (double)((System.currentTimeMillis() - start)/1000);

    Trajectory.State goal = trajectory.sample(time);
    ChassisSpeeds adjustedSpeeds = controller.calculate(drivetrain.getPose(), goal);

    drivetrain.setChassisSpeeds(adjustedSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) drivetrain.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, Rotation2d.fromDegrees(sensors.getGyro())));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (time == totalTime);
  }
}
