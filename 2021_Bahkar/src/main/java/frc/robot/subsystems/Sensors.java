/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.enums.LedMode;
import frc.robot.utils.components.LEDPanel;
import frc.robot.utils.enums.CameraMode;
import frc.robot.utils.enums.Pipeline;

/**
 * Class to control and monitor sensors.
 */
public class Sensors extends SubsystemBase {

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private final PigeonIMU pigeon = new PigeonIMU(31);

  private NetworkTable TlimelightTable = null;
  private NetworkTable ClimelightTable = null;
  private Pipeline pipeline = Pipeline.CLOSE;

  private final DigitalInput topBeamA = new DigitalInput(0);
  private final DigitalInput topBeamB = new DigitalInput(1);

  Notifier shuffle;
  //LEDPanel ledPanel;

  /**
   * Creates a new Sensors.
   */
  public Sensors(/*LEDPanel ledPanel*/) {
    //this.ledPanel = ledPanel;

    setupPigeon();
    setupGyro();

    TlimelightTable = NetworkTableInstance.getDefault().getTable("turret_limelight");
    ClimelightTable = NetworkTableInstance.getDefault().getTable("chassis_limelight");

    TlimelightTable.getEntry("pipeline").setNumber(pipeline.getValue());
    ClimelightTable.getEntry("pipeline").setNumber(pipeline.getValue());

    shuffle = new Notifier(() -> updateShuffleBoard());
  }

  @Override
  public void periodic() {
    updateShuffleBoard();
  }

  /**
   * Send updated values to NetworkTables; call in a Notifier
   */
  private void updateShuffleBoard() {
    SmartDashboard.putNumber("Pigeon Fused Heading", getFusedHeading());
    SmartDashboard.putBoolean("Top Beam A", topBeamA.get());
    SmartDashboard.putBoolean("Top Beam B", topBeamB.get());
  }

  public void closeNotifier() {
    shuffle.close();
  }

  public void openNotifier() {
    shuffle.startPeriodic(0.1);
  }

  /**
   * Prepares the gyro for competition.
   */
  private void setupGyro() {
    calibrateGyro();
    resetGyro();
  }

  /**
   * Prepared the pigeon for competition
   */
  private void setupPigeon() {
    pigeon.configFactoryDefault();
    pigeon.setYaw(0.0);
    pigeon.setFusedHeading(0.0);
    pigeon.setAccumZAngle(0.0);
  }

  /**
   * Return the more accurate fused heading of the pigeon.
   * @return Fused heading (normal gyro with corrections).
   */
  public double getFusedHeading() {
    return -Math.IEEEremainder(pigeon.getFusedHeading(), 360);
  }

  /**
   * Calibrate the gyro to fix variability in returns; requires a few seconds idle.
   */
  private void calibrateGyro() {
    gyro.calibrate();
  }

  /**
   * Send the gyro/pigeon to 0 degrees.
   */
  public void resetGyro() {
    gyro.reset();
    pigeon.setYaw(0.0);
    pigeon.setFusedHeading(0.0);
  }

  /**
   * value read from the Analog Devices Gyro (innacurate after very fast spinning).
   * @return Degree value of the gyro in degrees.
   */
  public double getGyro() {
    return Math.IEEEremainder(-gyro.getAngle(), 360);
  }

  /**
   * Pigeon gyro rotation value.
   * @return pige
   */
  public double getYaw() {
    double[] measures = {0,0,0}; 
    pigeon.getYawPitchRoll(measures);
    return -Math.IEEEremainder(measures[0], 360);
  }

  /**
   * Returns the pigeon yaw as a Rotation2d object.
   * @return Rotation2d measured by pigeon.
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getYaw());
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

  /**
   * Determines whether a limelight target is in view of the turret limelight.
   * @return True if there is a single limelight target, false otherwise.
   */
  public boolean getTurretHasTarget() {
    return (TlimelightTable.getEntry("tv").getDouble(0.0) == 1.0);
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
      ((x > 0.0) ? (0.0) : (180.0)
      ))));
  }

  public double[] getTurretXYAngle() {
    double ratio = (getTurretCenterOff() + (0.164 * getTurretCenterOff() + 0.102)) / getTurretCenterOff();
    double[] XY = {(getTurretTX() * ratio), (getTurretTY() * ratio)};
    return XY;
  }

  /**
   * Returns the limelight verticle angle correcting the inherent error.
   * @return Real angle from the target to the limelight, with correction.
   */
  public double getTurretVerticleAngle() {
    return getTurretXYAngle()[1] + Constants.Turret.MOUNTING_ANGLE;
  }

  /**
   * Returns the limelight verticle angle correcting the inherent error.
   * @return Real angle from the target to the limelight, with correction.
   */
  public double getTurretHorizontalAngle() {
    return getTurretXYAngle()[0];
  }

  /**
   * Return the distance from the robot to the goal; only for use on the hexagonal port goal.
   * @return distance from the robot to the goal in meters.
   */
  public double getDistance() {
    return getTurretHasTarget() ? (Constants.Field.GOAL_HEIGHT - Constants.Turret.LIMELIGHT_HEIGHT) / (Math.tan(Math.toRadians(getTurretVerticleAngle()))) : 0.0;
  }

  /**
   * Returns the correction to score in the inner port.
   * @return The angle offset from the target to aim at in order to hit inner port.
   */
  public double getTurretOffset() {
    double a = Constants.Field.THREE_POINT_DEPTH;
    double b = getDistance();
    double c = getTurretSkew();


    double offset =  Math.toDegrees(Math.asin((a * Math.sin(Math.toRadians(180-c))) / Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2) - (2 * a * b * Math.cos(Math.toRadians(180 - c))))));
    double check = c - offset;
    if (check <= Constants.Field.OFFSET_LIMIT) return offset;
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

  /**
   * Returns the chassis limelight verticle angle value corrected for 
   * @return
   */
  public double getChassisVerticleAngle() {
    return getChassisXYAngle()[1];
  }

  /**
   * Returns the chassis limelight horizontal angle value corrected for 
   * @return
   */
  public double getChassisHorizontalAngle() {
    return getChassisXYAngle()[0];
  }

  /**
   * Returns whether there in an obstruction in the top beam.
   * @return True if the beam in unbroken, false if it has been broken.
   */
  public boolean getTopBeam() {
    return (topBeamA.get() && topBeamB.get());
  }
}
