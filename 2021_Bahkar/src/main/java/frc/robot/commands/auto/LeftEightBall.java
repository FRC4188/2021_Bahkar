// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.drive.trajectorycontrol.CSPSwerveControl;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.hopper.AutoMagCount;
import frc.robot.commands.turret.TurretToOneEighty;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utils.CspSequentialCommandGroup;
import frc.robot.utils.trajectory.TrajectoryList;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LeftEightBall extends CspSequentialCommandGroup {
  /** Creates a new LeftEightBall. */
  public LeftEightBall(Drivetrain drivetrain, Intake intake, Hopper hopper, Shooter shooter, Turret turret, Hood hood){
    addCommands(
      new ParallelCommandGroup(
        new InstantCommand(() -> intake.lower(), intake),
        new TurretToOneEighty(turret)
      ),

      new AutoMagCount(hopper, shooter, hood, turret, 3),

      new ParallelDeadlineGroup(
        new CSPSwerveControl(drivetrain, TrajectoryList.LeftEightBall.PICK_UP),
        new AutoIntake(intake, hopper, true),
        new TurretToOneEighty(turret)
      ),

      new AutoIntake(intake, hopper, false),
      new AutoMagCount(hopper, shooter, hood, turret, 5)
    );
  }

  public Pose2d getInitialPose() {
    return TrajectoryList.TestAuto.motionOne.getTrajectory(0).getInitialPose();
  }
}
