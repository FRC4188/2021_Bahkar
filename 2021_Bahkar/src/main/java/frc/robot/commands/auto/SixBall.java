// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.ResetOdometry;
import frc.robot.commands.drive.Straight;
import frc.robot.commands.hood.SetPosition;
import frc.robot.commands.hopper.AutoMagazine;
import frc.robot.commands.hopper.SpinHopper;
import frc.robot.commands.sensors.ResetGyro;
import frc.robot.commands.shooter.SpinShooter;
import frc.robot.commands.turret.FollowTarget;
import frc.robot.commands.turret.TurretAngle;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Sensors;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.utils.trajectory.WaypointsList;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixBall extends SequentialCommandGroup {
  /** Creates a new SixBall. */
  public SixBall(Drivetrain drivetrain, Shooter shooter, Hopper hopper, Sensors sensors, Turret turret, Hood hood) {
    addCommands(
      // First reset the sensors and odometry.
        new ResetGyro(sensors),
        new ResetOdometry(drivetrain, WaypointsList.SixBall.INIT_POSE),

        // Second, aim the turret close to the target.
        new TurretAngle(turret, 30.0),
  
        // Then prepare the shooter.
        new SpinShooter(shooter, 4000),
        new SetPosition(hood, 0.77),
  
        // Auto aim the turret and fire.
        new ParallelCommandGroup(
          new FollowTarget(turret, true),
          new AutoMagazine(hopper, shooter, turret, true)
        ).withTimeout(5.0),
    
        // End the auto-aiming and shooting.
        new ParallelCommandGroup(
          new FollowTarget(turret, false),
          new SpinShooter(shooter, Constants.shooter.IDLE_VEL, false),
          new SpinHopper(hopper, 0.0, false)
        ).withTimeout(0.1),
  
        // Drive down the trench.
        new Straight(drivetrain, 4.75)
    );
  }
}
