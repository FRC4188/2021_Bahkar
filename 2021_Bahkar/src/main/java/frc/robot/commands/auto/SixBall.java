// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.drive.Reverse;
import frc.robot.commands.drive.Straight;
import frc.robot.commands.drive.trajectorycontrol.HolonomicControl;
import frc.robot.commands.groups.AutoIntake;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.groups.AutoShootQuantity;
import frc.robot.commands.hood.SetPosition;
import frc.robot.commands.hopper.AutoMagazine;
import frc.robot.commands.hopper.SpinHopper;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetGyro;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.turret.FollowTarget;
import frc.robot.commands.turret.TurretAngle;
import frc.robot.commands.turret.TurretPower;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utils.trajectory.WaypointsList;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixBall extends SequentialCommandGroup {
  /** Creates a new SixBall. */
  public SixBall(Drivetrain drivetrain, Shooter shooter, Hopper hopper, Intake intake, Sensors sensors, Turret turret, Hood hood) {
    addCommands(
      // First reset the sensors and odometry.
        new ResetGyro(sensors, drivetrain),
        new ResetOdometry(drivetrain, WaypointsList.SixBall.INIT_POSE),

        new InstantCommand(() -> intake.lower(), intake),

        // Then prepare the shooter.
        new SetPosition(hood, 0.75),

        new SpinShooter(shooter, 3500.0, false),
  
        // Auto aim the turret and fire.
        new AutoShoot(shooter, turret, hood, hopper, sensors, true, 3500.0).withTimeout(2.0),
        new AutoShoot(shooter, turret, hood, hopper, sensors, false, 3500.0),

        new AutoShoot(shooter, turret, hood, hopper, sensors, true, 3500.0).withTimeout(1.0),
        new AutoShoot(shooter, turret, hood, hopper, sensors, false, 3500.0),

        new ParallelDeadlineGroup(

          new SequentialCommandGroup(
            // Drive down the trench.
            new HolonomicControl(drivetrain, WaypointsList.SixBall.DOWN, new Rotation2d(Math.PI)),
            new HolonomicControl(drivetrain, WaypointsList.SixBall.BACK, new Rotation2d(Math.PI))
          ),
          // Begin intaking balls
          new AutoIntake(intake, hopper, sensors, true),
          new SetPosition(hood, 0.8)
        ),

        // Stop intaking balls.
        new AutoIntake(intake, hopper, sensors, false),

        new RunCommand(() -> drivetrain.drive(0.0, 0.0, 0.0, false), drivetrain).withTimeout(0.25),

        new AutoShoot(shooter, turret, hood, hopper, sensors, true, 3750.0).withTimeout(3.0),

        new AutoShoot(shooter, turret, hood, hopper, sensors, false, 3750.0),

        new ParallelCommandGroup(
          new SpinIntake(intake, 0.0),
          new SpinHopper(hopper, 0.0),
          new TurretPower(turret, 0.0),
          new SpinShooter(shooter, Constants.shooter.IDLE_VEL)
        )
    );
  }
}
