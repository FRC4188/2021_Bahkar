/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.robot.utils.components.WheelDrive;

public class Drivetrain extends SubsystemBase {

  private Sensors sensors;

  //Initialize WheelDrive objects
  private WheelDrive LeftFront = new WheelDrive(1, 2, 21, -138.691406, false, false);
  private WheelDrive RightFront = new WheelDrive(3, 4, 22, 171.562500, true, false);
  private WheelDrive LeftRear = new WheelDrive(5, 6, 23, 4.833984, false, true);
  private WheelDrive RightRear = new WheelDrive(7, 8, 24, 157.412109, true, true);

  //Create initial odometry
  private SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(new Rotation2d(), new Pose2d(), Constants.drive.KINEMATICS,
  VecBuilder.fill(0.5, 0.5, 0.5),
  VecBuilder.fill(0.2),
  VecBuilder.fill(0.0, 0.0, 0.0));

  private SwerveDriveKinematics kinematics = Constants.drive.KINEMATICS;

  //Initialize a ChassisSpeeds object and start it with default values
  private ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, new Rotation2d());

  private PIDController rotationPID = new PIDController(0.2, 0.0, 0.001);

  //Create a configuration for trajectories.
  private CentripetalAccelerationConstraint CentAccel = new CentripetalAccelerationConstraint(Constants.drive.auto.MAX_CACCEL);
  private TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.drive.auto.MAX_VELOCITY, Constants.drive.auto.MAX_ACCEL).addConstraint(CentAccel);

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain(Sensors sensors) {
    this.sensors = sensors;

    LeftFront.setInverted(false);
    RightFront.setInverted(false);
    LeftRear.setInverted(false);
    RightRear.setInverted(false);

    rotationPID.enableContinuousInput(-180, 180);

    SmartDashboard.putNumber("Correction P", 0.0);
    SmartDashboard.putNumber("Correction I", 0.0);
    SmartDashboard.putNumber("Correction D", 0.0);

    SmartDashboard.putNumber("Set Speed kP", 0.0);
    SmartDashboard.putNumber("Set Angle kP", 0.0);

    Notifier shuffle = new Notifier(() -> updateShuffleboard());
    shuffle.startPeriodic(0.1);
  }

  /**
   * This method will be called once every scheduler run.
   */
  @Override
  public void periodic() {
    updateOdometry();
  }

  boolean lastNoAngle = true;
  double Angle = 0.0;
  /**
   * Method for field oriented drive using kinematics
   * @param pilot CspController of the pilot
   */
  public void drive (double speed, double strafe, double rotation, boolean FO) {

    //Convert controller input to M/S and Rad/S
    speed = speed * Constants.drive.MAX_VELOCITY;
    strafe = strafe * Constants.drive.MAX_VELOCITY;

    rotation = rotation * Constants.drive.MAX_RADIANS;

    double currentAngle = sensors.getFusedHeading();

    double angleCorrection = 0.0;
    /*
    if (Rotation != 0) Angle = currentAngle;
    else if (Speed != 0 || Strafe != 0) rotationPID.calculate(currentAngle, Angle);
*/

    if(rotation != 0){
      Angle = currentAngle;
      }else{
      if( Math.abs(speed) > 0 || Math.abs(strafe) > 0 ){
      angleCorrection = rotationPID.calculate(currentAngle, Angle);
      }
      }

    boolean fieldRelative = !FO;

    //Get a chassis speed and rotation from input.
    ChassisSpeeds speeds = (fieldRelative) ? (ChassisSpeeds.fromFieldRelativeSpeeds(
      speed, strafe, rotation + angleCorrection, Rotation2d.fromDegrees(currentAngle))) :
      (new ChassisSpeeds(speed, strafe, rotation));

    //Set the Chassis speeds after processing input.
    setChassisSpeeds(speeds);
  }

  /**
   * Takes a ChassisSpeeds object and converts to to module states, then sets the modules to their states.
   * @param speeds The ChassisSpeeds object.
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    //get an array of module states from the ChassisSpeeds.
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.normalizeWheelSpeeds(states, Constants.drive.MAX_VELOCITY);

    setModuleStates(states);
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    //Seperate the elements of the array.
    SwerveModuleState frontLeft = moduleStates[0];
    SwerveModuleState frontRight = moduleStates[1];
    SwerveModuleState backLeft = moduleStates[2];
    SwerveModuleState backRight = moduleStates[3];

    //Set the modules in the WheelDrive objects to the kinematic results.
    LeftFront.convertedDrive(frontLeft);
    RightFront.convertedDrive(frontRight);
    LeftRear.convertedDrive(backLeft);
    RightRear.convertedDrive(backRight);
  }

  /**
   * Method to update the odometry of the robot.
   */
  public void updateOdometry() {
    //get the module state from each motor
    SwerveModuleState frontLeft = LeftFront.updateModuleState();
    SwerveModuleState frontRight = RightFront.updateModuleState();
    SwerveModuleState backLeft = LeftRear.updateModuleState();
    SwerveModuleState backRight = RightRear.updateModuleState();

    //update odometry using the new module states.
    odometry.update(Rotation2d.fromDegrees(sensors.getFusedHeading()), frontLeft, frontRight, backLeft, backRight);
    speeds = kinematics.toChassisSpeeds(new SwerveModuleState[] {frontLeft, frontRight, backLeft, backRight});
  }

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, sensors.getRotation2d());
  }

  /**p
   * Publish value from the drivetrain to the Smart Dashboard.
   */
  private void updateShuffleboard() {
    SmartDashboard.putString("Odometry", odometry.getEstimatedPosition().toString());
    SmartDashboard.putNumber("Left Front Angle", LeftFront.getAbsoluteAngle());
    SmartDashboard.putNumber("Right Front Angle", RightFront.getAbsoluteAngle());
    SmartDashboard.putNumber("Left Rear Angle", LeftRear.getAbsoluteAngle());
    SmartDashboard.putNumber("Right Rear Angle", RightRear.getAbsoluteAngle());
    SmartDashboard.putNumber("Chassis Velocity", Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2.0) + Math.pow(speeds.vyMetersPerSecond, 2.0)));

  }

  public void reset() {
    LeftFront.resetEncoders();
    RightFront.resetEncoders();
    LeftRear.resetEncoders();
    RightRear.resetEncoders();
  }


  public void setPIDs() {
    rotationPID.setPID(SmartDashboard.getNumber("Correction P", 0.0), SmartDashboard.getNumber("Correction I", 0.0), SmartDashboard.getNumber("Correction D", 0.0));
  }

  public void setAnglePID(double kP, double kI, double kD) {
    LeftFront.setAnglePID(kP, kI, kD);
    RightFront.setAnglePID(kP, kI, kD);
    LeftRear.setAnglePID(kP, kI, kD);
    RightRear.setAnglePID(kP, kI, kD);
  }

  public void setSpeedPID(double kP, double kI, double kD) {
    LeftFront.setSpeedPID(kP, kI, kD);
    RightFront.setSpeedPID(kP, kI, kD);
    LeftRear.setSpeedPID(kP, kI, kD);
    RightRear.setSpeedPID(kP, kI, kD);
  }


  /**
   * The object for converting motion to individual motor states.
   * @return The Kinematics object.
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public TrajectoryConfig getConfig() {
    return trajectoryConfig;
  }

  /**
   * The object for locating the robot on the field.
   * @return the Odometry object
   */
  public SwerveDrivePoseEstimator getOdometry() {
    return odometry;
  }

  /**
   * The object for converting a series of inputs to a single motion.
   * @return the ChassisSpeeds object.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return speeds;
  }

  /**
   * Get the current module states of the motors.
   * @return SwerveModuleStates array.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      LeftFront.updateModuleState(),
      RightFront.updateModuleState(),
      LeftRear.updateModuleState(),
      RightRear.updateModuleState()
    };
  }

  /**
   * Get the current position of the robot.
   * @return Pose2d position.
   */
  public Pose2d getPose() {
    return odometry.getEstimatedPosition();
  }

  public double getFrontLeftDriveTemp() {
    return LeftFront.getSpeedTemp();
  }
  public double getFrontLeftAngleTemp() {
    return LeftFront.getAngleTemp();
  }
  public double getFrontRightDriveTemp() {
    return RightFront.getSpeedTemp();
  }
  public double getFrontRightAngleTemp() {
    return RightFront.getAngleTemp();
  }
  public double getRearLeftDriveTemp() {
    return LeftRear.getSpeedTemp();
  }
  public double getRearLeftAngleTemp() {
    return LeftRear.getAngleTemp();
  }
  public double getRearRightDriveTemp() {
    return RightRear.getSpeedTemp();
  }
  public double getRearRightAngleTemp() {
    return RightRear.getAngleTemp();
  }
}