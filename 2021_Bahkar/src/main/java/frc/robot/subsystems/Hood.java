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

  Notifier shuffle;

  /**
   * Creates a new Hood.
   */
  public Hood() {
    SmartDashboard.putNumber("Set Hood Position", 0.0);
    servos.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

    shuffle = new Notifier(() -> updateShuffleboard());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sends updated values to NetworkTables.
   */
  private void updateShuffleboard() {
    SmartDashboard.putNumber("Hood Position", getPos());
  }

  public void closeNotifier() {
    shuffle.close();
  }

  public void openNotifier() {
    shuffle.startPeriodic(0.1);
  }

  /**
   * Set the position of the hood servos
   * @param pos Position in range [0.0, -1.0].
   */
  public void set(double pos) {
    servos.setPos(pos);
  }

  /**
   * Return the position which the servo is set to.
   * @return Set position in range [0.0, 1.0].
   */
  public double getPos() {
    return servos.getPos();
  }

  /**
   * Sets the hood position to stay where it is.
   */
  public void holdPos() {
    servos.setPos(servos.getPos());
  }
}
