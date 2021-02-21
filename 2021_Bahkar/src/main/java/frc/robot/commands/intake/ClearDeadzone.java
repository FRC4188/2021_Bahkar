/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;

public class ClearDeadzone extends CommandBase {
  Intake intake;
  Timer timer;
  int flops = 0;
  double lastRef = 0.0;
  /**
   * Creates a new ClearDeadzone.
   */
  public ClearDeadzone(Intake intake) {
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    intake.lower();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double current = timer.get() % (Constants.Intake.FLOP_RATE / 2);
  
    if (current < lastRef) {
      intake.toggle();
      flops = !intake.getIsLowered() ? flops + 1 : flops;
    }

    lastRef = current;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.lower();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return flops == 4;
  }
}
