/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CSPMath;
import frc.robot.utils.components.DualServos;
import frc.robot.utils.components.LinearActuator;

public class Hood extends SubsystemBase {
  DualServos servos = new DualServos(new LinearActuator(7), new LinearActuator(8));

  Sensors sensors;
  Drivetrain drivetrain;

  Notifier shuffle;

  /**
   * Creates a new Hood.
   */
  public Hood(Sensors sensors, Drivetrain drivetrain) {
    SmartDashboard.putNumber("Set Hood Position", 0.0);

    shuffle = new Notifier(() -> updateShuffleboard());
    shuffle.startPeriodic(0.1);

    this.sensors = sensors;
    this.drivetrain = drivetrain;
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

  public void setAngle(double angle) {
    servos.setPos(CSPMath.Hood.angleToSet(angle));
  }

  /**
   * Finds the correct angle for the shooter for the current zone's RPM.
   * @return Hood angle. degrees.
   */
  public double formulaAngle() {
    return 0.0;
  }

  public void setFormulaAngle() {
    setAngle(formulaAngle());
  }

  /**
   * Return the position which the servo is set to.
   * @return Set position in range [0.0, 1.0].
   */
  public double getPos() {
    return servos.getPos();
  }

  public double getAngle() {
    return CSPMath.Hood.setToAngle(servos.getPos());
  }

  /**
   * Sets the hood position to stay where it is.
   */
  public void holdPos() {
    servos.setPos(servos.getPos());
  }

  public boolean isAimed() {
    return getPos() - formulaAngle() == 0.0;
  }
}
