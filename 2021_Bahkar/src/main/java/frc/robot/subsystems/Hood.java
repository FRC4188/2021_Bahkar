/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.components.DualServos;

public class Hood extends SubsystemBase {
  DualServos servos = new DualServos(new Servo(0), new Servo(1));

  /**
   * Creates a new Hood.
   */
  public Hood() {
    SmartDashboard.putNumber("Set Hood Position", 0.0);
    servos.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    Notifier shuffle = new Notifier(() -> updateShuffleboard());
    shuffle.startPeriodic(0.1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Hood Position", getPos());
  }

  public void set(double pos) {
    servos.setPos(pos);
  }

  public double getPos() {
    return servos.getPos();
  }

  public void holdPos() {
    servos.setPos(servos.getPos());
  }
}
