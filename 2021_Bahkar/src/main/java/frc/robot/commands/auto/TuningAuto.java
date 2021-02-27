// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.drive.trajectorycontrol.HolonomicControl;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.trajectory.TrajectoryList;
import frc.robot.utils.trajectory.WaypointsList;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TuningAuto extends SequentialCommandGroup {
  /** Creates a new TuningAuto. */
  public TuningAuto(Drivetrain drivetrain) {
    addCommands(
      new ResetOdometry(drivetrain, WaypointsList.tuning.INIT_POSE),
      new HolonomicControl(drivetrain, TrajectoryList.TuningAuto.meter.getTrajectory(0), TrajectoryList.TuningAuto.meter.getAngle(0))
      .andThen(new InstantCommand(() -> drivetrain.setChassisSpeeds(new ChassisSpeeds())))
    );
  }
}
