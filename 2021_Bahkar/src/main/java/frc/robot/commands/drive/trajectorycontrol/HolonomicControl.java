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
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class HolonomicControl extends CommandBase {
  Drivetrain drivetrain;
  Trajectory trajectory;
  Rotation2d angle;
  int index = 0;

  Timer timer = new Timer();

  ProfiledPIDController thetaController = new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(Constants.drive.MAX_RADIANS, 2.0 * Constants.drive.MAX_RADIANS));
  PIDController xController = new PIDController(1.0, 0.0, 0.0);
  PIDController yController = new PIDController(1.0, 0.0, 0.0);

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
    controller.setTolerance(new Pose2d(0.1, 0.1, new Rotation2d(0.1)));
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
    drivetrain.setChassisSpeeds(controller.calculate(pose, trajectory.sample(timer.get()), angle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() == trajectory.getTotalTimeSeconds() && controller.atReference();
  }
}
