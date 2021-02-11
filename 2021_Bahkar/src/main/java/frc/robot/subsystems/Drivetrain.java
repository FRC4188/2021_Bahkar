/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.controller.PIDController;
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
import frc.robot.Constants;
import frc.robot.utils.components.WheelDrive;

public class Drivetrain extends SubsystemBase {

  private Sensors sensors;

  //Initialize WheelDrive objects
  private WheelDrive LeftFront = new WheelDrive(1, 2, 21, 73.125, false, false);
  private WheelDrive RightFront = new WheelDrive(3, 4, 22, 172.7, true, false);
  private WheelDrive LeftRear = new WheelDrive(5, 6, 23, 4.3, false, true);
  private WheelDrive RightRear = new WheelDrive(7, 8, 24, 158.37, true, true);

  //Put together swerve module positions relative to the center of the robot.
  private Translation2d FrontLeftLocation = new Translation2d((Constants.Robot.A_LENGTH/2), -(Constants.Robot.A_WIDTH/2));
  private Translation2d FrontRightLocation = new Translation2d((Constants.Robot.A_LENGTH/2), (Constants.Robot.A_WIDTH/2));
  private Translation2d BackLeftLocation = new Translation2d(-(Constants.Robot.A_LENGTH/2), -(Constants.Robot.A_WIDTH/2));
  private Translation2d BackRightLocation = new Translation2d(-(Constants.Robot.A_LENGTH/2), (Constants.Robot.A_WIDTH/2));

  //Create a kinematics withe the swerve module positions
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    FrontLeftLocation, FrontRightLocation, BackLeftLocation, BackRightLocation);

  //Initialize a ChassisSpeeds object and start it with default values
  private ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, new Rotation2d());

  private PIDController rotationPID = new PIDController(0.2, 0.0, 0.001);

  //Initialize a list of module states and assign the kinematic results to them
  private SwerveModuleState[]  moduleStates = kinematics.toSwerveModuleStates(speeds);

  //Assign module states to modules
  private SwerveModuleState frontLeft = moduleStates[0];
  private SwerveModuleState frontRight = moduleStates[1];
  private SwerveModuleState backLeft = moduleStates[2];
  private SwerveModuleState backRight = moduleStates[3];

  //Create initial odometry
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
  new Rotation2d(), new Pose2d(0.0, 0.0, new Rotation2d()));

  //Create a configuration for trajectories.
  private CentripetalAccelerationConstraint CentAccel = new CentripetalAccelerationConstraint(Constants.Drive.Auto.MAX_CACCEL);
  private TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.Drive.Auto.MAX_VELOCITY, Constants.Drive.Auto.MAX_ACCEL).addConstraint(CentAccel);

  Notifier shuffle;
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

    shuffle = new Notifier(() -> updateShuffleboard());
  }

  /**
   * This method will be called once every scheduler run.
   */
  @Override
  public void periodic() {
    updateOdometry();
  }

  double Angle;
  /**
   * Method for field oriented drive using kinematics
   * @param pilot CspController of the pilot
   */
  public void drive (double speed, double strafe, double rotation, boolean FO) {

    //Convert controller input to M/S and Rad/S
    speed = speed * Constants.Drive.MAX_VELOCITY;
    strafe = strafe * Constants.Drive.MAX_VELOCITY;

    rotation = rotation * Constants.Drive.MAX_RADIANS;

    double currentAngle = sensors.getFusedHeading();

    double angleCorrection = 0.0;

    // When moving in plane without rotation, holds the angle at the last set angle.
    if(rotation != 0){
      Angle = currentAngle;
    }else{
      if( Math.abs(speed) > 0 || Math.abs(strafe) > 0 ){
        angleCorrection = rotationPID.calculate(currentAngle, Angle);
      }
    }

    boolean fieldRelative = !FO;

    //Get a chassis speed and rotation from input.
    speeds = (fieldRelative) ? (ChassisSpeeds.fromFieldRelativeSpeeds(
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
    moduleStates = kinematics.toSwerveModuleStates(speeds);

    setModuleStates(moduleStates);
  }

  public void setModuleStates(SwerveModuleState[] moduleStates) {
    //Seperate the elements of the array.
    frontLeft = moduleStates[0];
    frontRight = moduleStates[1];
    backLeft = moduleStates[2];
    backRight = moduleStates[3];

    //Set the modules in the WheelDrive objects to the kinematic results.
    LeftFront.convertedDrive(frontLeft);
    RightFront.convertedDrive(frontRight);
    LeftRear.convertedDrive(backLeft);
    RightRear.convertedDrive(backRight);
  }

  /**
   * Set the angle and speed of the swerve modules (for testing and diagnostic purposes).
   * @param angle The angle to set the modules to in degrees within the range [-180.0, 180.0].
   * @param velocity The speed to set the motor to in meters per second.
   */
  public void rawSet(double angle, double velocity) {
    LeftFront.setAngle(angle);
    RightFront.setAngle(angle);
    LeftRear.setAngle(angle);
    RightRear.setAngle(angle);

    LeftFront.setVelocity(velocity);
    RightFront.setVelocity(velocity);
    LeftRear.setVelocity(velocity);
    RightRear.setVelocity(velocity);
  }

  /**
   * Method to update the odometry of the robot.
   */
  public void updateOdometry() {
    //get the module state from each motor
    frontLeft = LeftFront.updateModuleState();
    frontRight = RightFront.updateModuleState();
    backLeft = LeftRear.updateModuleState();
    backRight = RightRear.updateModuleState();

    //update odometry using the new module states.
    odometry.update(Rotation2d.fromDegrees(sensors.getFusedHeading()), frontLeft, frontRight, backLeft, backRight);
  }

  /**
   * Reset the odometry to a given position.
   * @param pose Position to set the odometry to.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, Rotation2d.fromDegrees(sensors.getFusedHeading()));
  }

  /**
   * Publish value from the drivetrain to the Smart Dashboard.
   */
  private void updateShuffleboard() {
    SmartDashboard.putString("Odometry", odometry.getPoseMeters().toString());
    SmartDashboard.putString("ChassisSpeeds", getChassisSpeeds().toString());
    SmartDashboard.putNumber("Total Speed", Math.sqrt( Math.pow(getChassisSpeeds().vxMetersPerSecond, 2) + Math.pow(getChassisSpeeds().vyMetersPerSecond, 2)));
  }

  public void closeNotifier() {
    shuffle.close();
  }

  public void openNotifier() {
    shuffle.startPeriodic(0.1);
  }

  /**
   * Reset all of the swerve modules' encoders.
   */
  public void reset() {
    LeftFront.resetEncoders();
    RightFront.resetEncoders();
    LeftRear.resetEncoders();
    RightRear.resetEncoders();
  }

  /**
   * The object for converting motion to individual motor states.
   * @return The Kinematics object.
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * The object for locating the robot on the field.
   * @return the Odometry object
   */
  public SwerveDriveOdometry getOdometry() {
    return odometry;
  }

  /**
   * The object for converting a series of inputs to a single motion.
   * @return the ChassisSpeeds object.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Get the current module states of the motors.
   * @return SwerveModuleStates array.
   */
  public SwerveModuleState[] getModuleStates() {
    return moduleStates;
  }

  /**
   * Get the current position of the robot.
   * @return Pose2d position.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Get the Trajectory config object.
   * @return TrajectoryConfig object.
   */
  public TrajectoryConfig getConfig() {
    return trajectoryConfig;
  }

  /**
   * Return the Front-Left speed motor's temperature.
   * @return Temperature of the motor in celsius.
   */
  public double getFrontLeftDriveTemp() {
    return LeftFront.getSpeedTemp();
  }

  /**
   * Return the Front-Left angle motor's temperature.
   * @return Temperature of the motor in celsius.
   */
  public double getFrontLeftAngleTemp() {
    return LeftFront.getAngleTemp();
  }

  /**
   * Return the Front-Right speed motor's temperature.
   * @return Temperature of the motor in celsius.
   */
  public double getFrontRightDriveTemp() {
    return RightFront.getSpeedTemp();
  }

  /**
   * Return the Front-Right angle motor's temperature.
   * @return Temperature of the motor in celsius.
   */
  public double getFrontRightAngleTemp() {
    return RightFront.getAngleTemp();
  }

  /**
   * Return the Rear-Left speed motor's temperature.
   * @return Temperature of the motor in celsius.
   */
  public double getRearLeftDriveTemp() {
    return LeftRear.getSpeedTemp();
  }

  /**
   * Return the Rear-Left angle motor's temperature.
   * @return Temperature of the motor in celsius.
   */
  public double getRearLeftAngleTemp() {
    return LeftRear.getAngleTemp();
  }

  /**
   * Return the Rear-Right speed motor's temperature.
   * @return Temperature of the motor in celsius.
   */
  public double getRearRightDriveTemp() {
    return RightRear.getSpeedTemp();
  }
  
  /**
   * Return the Rear-Right angle motor's temperature.
   * @return Temperature of the motor in celsius.
   */
  public double getRearRightAngleTemp() {
    return RightRear.getAngleTemp();
  }
}