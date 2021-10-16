// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.drive.Reverse;
import frc.robot.commands.drive.Straight;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.commands.hood.SetPosition;
import frc.robot.commands.hopper.SpinHopper;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.commands.sensors.ResetGyro;
import frc.robot.commands.sensors.ResetOdometry;
import frc.robot.commands.shooter.ShooterVelocity;
import frc.robot.commands.turret.TurretAngle;
import frc.robot.commands.turret.TurretPower;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SixBall extends SequentialCommandGroup {
  /** Creates a new SixBall. */
  public SixBall() {
    Intake intake = Intake.getInstace();
    Swerve drive = Swerve.getInstance();

    addCommands(
      // First reset the sensors and odometry.
        new ResetGyro(),
        new ResetOdometry(),

        new ShooterVelocity(4000, true),

        // Second, aim the turret close to the target.
        new TurretAngle(30.0),
  
        // Then prepare the shooter.
        new SetPosition(true),
  
        // Auto aim the turret and fire.
        new AutoShoot(true).withTimeout(5.0),
    
        // End the auto-aiming and shooting.
        new ParallelCommandGroup(
          new AutoShoot(false),
          new ShooterVelocity(Constants.shooter.IDLE_VEL, true)
        ),

        new InstantCommand(() -> intake.setRaised(false), intake),
  
        new ParallelDeadlineGroup(
          // Drive down the trench.
          new Straight(4.5),
          // Begin intaking balls
          new SpinIntake(0.7, true)
        ),

        // Stop intaking balls.
        new SpinIntake(0.0, false),

        new Reverse(4.5).andThen(new InstantCommand(() -> drive.setChassisSpeeds(new ChassisSpeeds()), drive)),

        new AutoShoot(true).withTimeout(5.0),

        new ParallelCommandGroup(
          new AutoShoot(false),
          new ShooterVelocity(Constants.shooter.IDLE_VEL, true)
        ),

        new ParallelCommandGroup(
          new SpinIntake(0.0, false),
          new SpinHopper(0.0, false),
          new TurretPower(0.0),
          new ShooterVelocity(Constants.shooter.IDLE_VEL, true)
        )
    );
  }
}
