/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
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
import frc.robot.utils.CSPMath;
import frc.robot.utils.enums.CameraMode;
import frc.robot.utils.enums.Pipeline;

public class Sensors extends SubsystemBase {

  private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
  private final PigeonIMU pigeon = new PigeonIMU(31);

  private NetworkTable TlimelightTable = null;
  private NetworkTable ClimelightTable = null;
  private Pipeline pipeline = Pipeline.CLOSE;

  private final DigitalInput topBeamA = new DigitalInput(0);
  private final DigitalInput topBeamB = new DigitalInput(1);


  boolean adjustedGyro = false;


  /**
   * Creates a new Sensors.
   */
  public Sensors() {
    setupPigeon();
    setupGyro();

    TlimelightTable = NetworkTableInstance.getDefault().getTable("turret_limelight");
    ClimelightTable = NetworkTableInstance.getDefault().getTable("chassis_limelight");

    TlimelightTable.getEntry("pipeline").setNumber(pipeline.getValue());
    ClimelightTable.getEntry("pipeline").setNumber(pipeline.getValue());

    Notifier shuffle = new Notifier(() -> updateShuffleBoard());
    shuffle.startPeriodic(0.1);

    /*
    // Creates UsbCamera and MjpegServer [1] and connects them
    CameraServer.getInstance().startAutomaticCapture();
    // Creates the CvSink and connects it to the UsbCamera
    CvSink cvSink = CameraServer.getInstance().getVideo();
    // Creates the CvSource and MjpegServer [2] and connects them
    CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);
    */
  }

  @Override
  public void periodic() {
    updateShuffleBoard();
  }

  private void updateShuffleBoard() {
    SmartDashboard.putNumber("Compass Heading", getCompassAngle());
    SmartDashboard.putNumber("Gyro Heading", getGyro());
    SmartDashboard.putNumber("Pigeon Yaw", getYaw());
    SmartDashboard.putNumber("Pigeon Fused Heading", getFusedHeading());
    //SmartDashboard.putNumber("Average of Pigeon, Compass, and Gyro measures.", getRotation());
    SmartDashboard.putBoolean("Top Forward Beam", topBeamA.get());
    SmartDashboard.putBoolean("Top Backward Beam", topBeamB.get());
  }

  private void setupGyro() {
    calibrateGyro();
    resetGyro();
  }

  
  private void setupPigeon() {
    pigeon.configFactoryDefault();
    pigeon.setCompassAngle(0);
    pigeon.setYaw(0.0);
    pigeon.setFusedHeading(0.0);
    pigeon.setAccumZAngle(0.0);
  }

  public double getFusedHeading() {
    return -Math.IEEEremainder(pigeon.getFusedHeading(), 360);
  }

  public double getCompassAngle() {
    return pigeon.getCompassHeading();
  }
  

  private void calibrateGyro() {
    gyro.calibrate();
  }

  public void resetGyro() {
    gyro.reset();
    pigeon.setYaw(0.0);
    pigeon.setFusedHeading(0.0);
  }

  public void fixGyro() {
    resetGyro();
    adjustedGyro = true;
  }

  public double getGyro() {
    return Math.IEEEremainder(gyro.getAngle(), 360) * -1;
  }

  
  public double getYaw() {
    double[] measures = {0,0,0}; 
    pigeon.getYawPitchRoll(measures);
    return -Math.IEEEremainder(measures[0], 360);
  }

  public double getRotation() {
    return (getYaw() + getGyro() + getCompassAngle()) / 3.0;
  }


  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getGyro());
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
    return getTurretXYAngle()[1] + Constants.Turret.MOUNTING_ANGLE;
  }

  public double getTurretHorizontalAngle() {
    return getTurretXYAngle()[0];
  }

  public double getDistance() {
    return (Constants.Field.GOAL_HEIGHT - Constants.Turret.LIMELIGHT_HEIGHT) / (Math.tan(Math.toRadians(getTurretVerticleAngle())));
  }

  public static double getHorizontalDistance() {
    return (1.0); 
  }

  public double[] formulaRPMandAngle() {
    double vy = CSPMath.getVy();
    double vx = CSPMath.getVx(getHorizontalDistance(), vy);
    double launchAngle = CSPMath.getLaunchAngle(vx, vy);
    double[] angleAndRPM = {CSPMath.getVelocity(vx, vy, launchAngle), CSPMath.getLaunchAngle(vx, vy)};

    return angleAndRPM;
  }

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

  public double getChassisVerticleAngle() {
    return getChassisXYAngle()[1];
  }

  public double getChassisHorizontalAngle() {
    return getChassisXYAngle()[0];
  }

  public boolean getTopBeam() {
    return (topBeamA.get() && topBeamB.get());
  }
}
