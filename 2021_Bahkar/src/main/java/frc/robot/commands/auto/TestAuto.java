// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.groups.AutoShoot;
import frc.robot.utils.Trajectories;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAuto extends SequentialCommandGroup {
  /** Creates a new TestAuto. */
  public TestAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ResetGyro(),
      new ResetOdometry(Trajectories.trench8L.POSE1),
      new AutoShoot(3500.0, true).withTimeout(3.5),
      new AutoShoot(0.0, false),

      new ParallelDeadlineGroup(
        new FollowTrajectory(Trajectories.TestAuto.First),
        new AutoIntake(true)),


        new FollowTrajectory(Trajectories.TestAuto.Second),
        new AutoShoot(3500.0, true).withTimeout(3.5),
        new AutoShoot(false),
        new ParallelDeadlineGroup(
          new SpinIntake(0.0, false),
          new SpinHopper(0.0, false),
          new TurretPower(0.0),
          new ShooterVelocity(Constants.shooter.IDLE_VEL, true)));
  }
}
