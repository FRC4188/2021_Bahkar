// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.hood.SetPosition;
import frc.robot.commands.hopper.AutoMagazine;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.turret.FollowTarget;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShoot extends ParallelCommandGroup {
  /** Creates a new AutoShoot. */
  public AutoShoot(Shooter shooter, Turret turret, Hood hood, Hopper hopper, Intake intake, boolean cont) {
    addCommands(
      new AutoMagazine(hopper, shooter, turret, cont),
      new SpinIntake(intake, 0.75, cont),
      new FollowTarget(turret, cont),
      new SpinShooter(shooter, 4000.0, cont),
      new SetPosition(hood, 0.82)
    );
  }
}
