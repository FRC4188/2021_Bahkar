/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.hopper.AutoHopper;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class AutoIntake extends ParallelCommandGroup {
  /**
   * Creates a new AutoIntake.
   */
  public AutoIntake(Intake intake, Hopper hopper, boolean cont) {
    super(
      new SpinIntake(intake, 1.0, cont),
      new AutoHopper(hopper, cont)
    );
  }
}
