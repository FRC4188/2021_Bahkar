// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.hood.SetPosition;
import frc.robot.commands.hopper.AutoMagQuantity;
import frc.robot.commands.hopper.AutoMagazine;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.turret.FollowTarget;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShootQuantity extends ParallelRaceGroup {
  /** Creates a new AutoShoot. */
  public AutoShootQuantity(Shooter shooter, Turret turret, Hood hood, Hopper hopper, Sensors sensors, int quantity) {
    addCommands(
      new AutoMagQuantity(hopper, shooter, turret, sensors, quantity),
      new FollowTarget(turret, true),
      new SpinShooter(shooter, 4000.0, true)
    );
  }

  public AutoShootQuantity(Shooter shooter, Turret turret, Hood hood, Hopper hopper, Sensors sensors, int quantity, double rpm) {
    addCommands(
      new AutoMagQuantity(hopper, shooter, turret, sensors, quantity, rpm),
      new FollowTarget(turret, true),
      new SpinShooter(shooter, rpm, true)
    );
  }
}
