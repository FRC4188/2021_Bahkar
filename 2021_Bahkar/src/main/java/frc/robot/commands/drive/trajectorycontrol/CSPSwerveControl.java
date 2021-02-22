// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive.trajectorycontrol;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.CspSequentialCommandGroup;
import frc.robot.utils.trajectory.CSPSwerveTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CSPSwerveControl extends CspSequentialCommandGroup {
  Pose2d initPose;

  /** Creates a new CSPSwerveControl. */
  public CSPSwerveControl(Drivetrain drivetrain, CSPSwerveTrajectory trajectory) {
    initPose = trajectory.getTrajectory(0).getInitialPose();

    for (int i = 0; i < trajectory.getLength(); i++) {
      if (i == trajectory.getLength() - 1) {
        addCommands(new HolonomicControl(drivetrain, trajectory.getTrajectory(i), trajectory.getAngle(i))
        .andThen(new InstantCommand(() -> drivetrain.setChassisSpeeds(new ChassisSpeeds()), drivetrain)));
      } else addCommands(new HolonomicControl(drivetrain, trajectory.getTrajectory(i), trajectory.getAngle(i)));
    }
  }
  
  @Override
  public Pose2d getInitialPose() {
    return initPose;
  }
}
