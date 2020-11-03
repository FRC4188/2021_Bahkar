package frc.robot.commands.autogroups;

import frc.robot.commands.drive.FollowTrajectory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Sensors;
import frc.robot.utils.CspSequentialCommandGroup;
import frc.robot.utils.trajectory.SpontaneousTrajectory;

public class SpontaneousToShoot extends CspSequentialCommandGroup {
  /**
   * Creates a new OneMeterTestGroup.
   */
  public SpontaneousToShoot(Drivetrain drivetrain, Sensors sensors) {

    SpontaneousTrajectory sts = new SpontaneousTrajectory(drivetrain.getPose(), drivetrain.getConfig());

    addCommands(
        new FollowTrajectory(drivetrain, sensors, sts.getTrajectory()
        )
    );
  }
}
