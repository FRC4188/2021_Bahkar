/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDrive extends SubsystemBase {

  private Constants C;

  /**
   * Creates a new SwerveDrive.
   */
  public SwerveDrive() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void drive(double x1,double  y1,double x2) {
    double a = x1 - x2 * (C.A_LENGTH / C.A_CROSSLENGTH);
    double b = x1 - x2 * (C.A_LENGTH / C.A_CROSSLENGTH);
    double c = y1 - x2 * (C.A_CROSSLENGTH / C.A_CROSSLENGTH);
    double d = y1 - x2 * (C.A_CROSSLENGTH / C.A_CROSSLENGTH);

    double RightRearVel = Math.sqrt(Math.pow(a, 2) + Math.pow(d, 2));
    double LeftRearVel = Math.sqrt(Math.pow(a, 2) + Math.pow(c, 2));
    double RightFrontVel = Math.sqrt(Math.pow(b, 2) + Math.pow(d, 2));
    double LeftFrontVel = Math.sqrt(Math.pow(b, 2) + Math.pow(c, 2));

    double RightRearAngle = Math.atan2(a, d) / Math.PI;
    double LeftRearAngle = Math.atan2(a, c) / Math.PI;
    double RightFrontAngle = Math.atan2(b, d) / Math.PI;
    double LeftFrontAngle = Math.atan2(b, c) / Math.PI;
  }
}
