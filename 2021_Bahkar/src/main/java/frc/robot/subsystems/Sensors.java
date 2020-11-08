/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.enums.LedMode;
import frc.robot.utils.enums.CameraMode;
import frc.robot.utils.enums.Pipeline;

public class Sensors extends SubsystemBase {

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

  private NetworkTable TlimelightTable = null;
  private NetworkTable ClimelightTable = null;
  private Pipeline pipeline = Pipeline.CLOSE;

  double GyroAdjust = 0.0;


  /**
   * Creates a new Sensors.
   */
  public Sensors() {
    calibrateGyro();
    resetGyro();
    TlimelightTable = NetworkTableInstance.getDefault().getTable("turret_limelight");
    ClimelightTable = NetworkTableInstance.getDefault().getTable("chassis_limelight");

    TlimelightTable.getEntry("pipeline").setNumber(pipeline.getValue());
    ClimelightTable.getEntry("pipeline").setNumber(pipeline.getValue());

    // Creates UsbCamera and MjpegServer [1] and connects them
    CameraServer.getInstance().startAutomaticCapture();
    // Creates the CvSink and connects it to the UsbCamera
    CvSink cvSink = CameraServer.getInstance().getVideo();
    // Creates the CvSource and MjpegServer [2] and connects them
    CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void calibrateGyro() {
    gyro.calibrate();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public double getGyro() {
    return -gyro.getAngle() + GyroAdjust;
  }

  /**
  * Sets the LED mode of the camera.
  * @param mode - the LED mode to set the camera to
  */
  public void setLightMode(LedMode mode) {
    TlimelightTable.getEntry("ledMode").setNumber(mode.getValue());
  }

  /**
   * Sets the camera mode of the camera.
   * @param mode - the camera mode to set the camera to
   */
  public void setCameraMode(CameraMode mode) {
    TlimelightTable.getEntry("camMode").setNumber(mode.getValue());
  }

  /**
   * Sets the pipeline of the camera.
   * @param pl - the camera mode to set the camera to
   */
  public void setPipeline(Pipeline pl) {
    TlimelightTable.getEntry("pipeline").setNumber(pl.getValue());
    pipeline = pl;
  }

  public double getTurretTY() {
    return TlimelightTable.getEntry("ty").getDouble(0.0);
  }

  public double getTurretTX() {
    return TlimelightTable.getEntry("tx").getDouble(0.0);
  }

  public double getTurretSkew() {
    double rawVal = TlimelightTable.getEntry("ts").getDouble(0.0);

    rawVal *= -4;
    rawVal = (rawVal > 180) ? (180-rawVal) : rawVal;

    return rawVal;
  }

  public boolean getTurretHasTarget() {
    boolean HasTarget = (TlimelightTable.getEntry("tv").getDouble(0.0) == 1.0) ? true : false;
    return HasTarget;
  }

  public double getTurretCenterOff() {
    return Math.sqrt(Math.pow(getTurretTX(), 2) + Math.pow(getTurretTY(), 2));
  }

  public double getTurretCenterAngle() {
    double x = getTurretTX();
    double y = getTurretTY();

    return Math.toDegrees(
      ((y != 0) ? (Math.atan(x / y)) : 
      ((x == 0.0) ? (0.0) :
      ((x > 0.0) ? (90.0) : (170.0)
      ))));
  }

  public double[] getTurretXYAngle() {
    double ratio = (getTurretCenterOff() + (0.164 * getTurretCenterOff() + 0.102)) / getTurretCenterOff();
    double[] XY = {(getTurretTX() * ratio), (getTurretTY() * ratio)};
    return XY;
  }

  public double getTurretVerticleAngle() {
    return getTurretXYAngle()[1] + Constants.TURRET_MOUNTING_ANGLE;
  }

  public double getTurretHorizontalAngle() {
    return getTurretXYAngle()[0];
  }

  public double getDistance() {
    return (Constants.GOAL_HEIGHT - Constants.TURRET_LIMELIGHT_HEIGHT) / (Math.tan(Math.toRadians(getTurretVerticleAngle())));
  }

  public double getTurretOffset() {
    double a = Constants.THREE_POINT_DEPTH;
    double b = getDistance();
    double c = getTurretSkew();


    double offset =  Math.toDegrees(Math.asin((a * Math.sin(Math.toRadians(180-c))) / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2) - (2 * a * b * Math.cos(Math.toRadians(180 - c))))));
    double check = c - offset;
    if (check <= Constants.OFFSET_LIMIT) return offset;
    else return 0.0;
  }

  public double getChassisTY() {
    return ClimelightTable.getEntry("ty").getDouble(0.0);
  }

  public double getChassisTX() {
    return ClimelightTable.getEntry("tx").getDouble(0.0);
  }

  public double getChassisSkew() {
    double rawVal =  ClimelightTable.getEntry("ts").getDouble(0.0);

    rawVal *= -4;
    rawVal = (rawVal > 180) ? (180-rawVal) : rawVal;

    return rawVal;  }

  public boolean getChassisHasTarget() {
    boolean HasTarget = (ClimelightTable.getEntry("tv").getDouble(0.0) == 1.0) ? true : false;
    return HasTarget;
  }

  public double getChassisCenterOff() {
    return Math.sqrt(Math.pow(getChassisTX(), 2) + Math.pow(getChassisTY(), 2));
  }

  public double getChassisCenterAngle() {
    double x = getChassisTX();
    double y = getChassisTY();

    return Math.toDegrees(
      ((y != 0) ? (Math.atan(x / y)) : 
      ((x == 0.0) ? (0.0) :
      ((x > 0.0) ? (90.0) : (170.0)
      ))));  }

  public double[] getChassisXYAngle() {
    double ratio = (getChassisCenterOff() + (0.164*getChassisCenterOff() + 0.102)) / getChassisCenterOff();
    double[] XY = {(getChassisTX() * ratio), (getChassisTY() * ratio)};
    return XY;
  }

  public double getChassisVerticleAngle() {
    return getChassisXYAngle()[1];
  }

  public double getChassisHorizontalAngle() {
    return getChassisXYAngle()[0];
  }
}
