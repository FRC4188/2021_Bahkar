// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeThrottleSpeed extends CommandBase {

   private Intake intake;
   private DoubleSupplier throttle;
   private boolean cont;

  /** Creates a new IntakeThrottleSpeed. */
  public IntakeThrottleSpeed(Intake intake, DoubleSupplier throttle, boolean cont) {
    addRequirements(intake);

    this.intake = intake;
    this.throttle = throttle;
    this.cont = cont;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.set(throttle.getAsDouble());
    SmartDashboard.putNumber("Intake Throttle Power", throttle.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.set(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return cont;
  }
}
