/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive.test;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class WheelRotationTest extends CommandBase {
  Drivetrain drivetrain;

  double start;

  /**
   * Creates a new WheelRotationTest.
   */
  public WheelRotationTest(Drivetrain drivetrain) {
    addRequirements(drivetrain);

    this.drivetrain = drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.reset();
    start = (double)((System.currentTimeMillis()));

    SmartDashboard.putNumber("Angle Setpoint", 90.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    double time = (double)((System.currentTimeMillis() - start)/10000);

    if (Math.round(time/2) % 2 <= 1) drivetrain.rawSet(0, 0);
    else drivetrain.rawSet(90, 0);
    */
    double set = SmartDashboard.getNumber("Angle Setpoint", 90.0);

    drivetrain.rawSet(set, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.rawSet(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
