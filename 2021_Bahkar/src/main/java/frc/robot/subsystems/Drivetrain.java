/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID.Hand;
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
import frc.robot.utils.CspController;
import frc.robot.utils.WheelDrive;
import frc.robot.utils.CspController.Scaling;

public class Drivetrain extends SubsystemBase {

  // device initialization
  private final TalonFX LFAngleMotor = new TalonFX(1);
  private final TalonFX LFSpeedMotor = new TalonFX(2);
  private final CANCoder LFangleEncoder = new CANCoder(21);

  private final TalonFX RFAngleMotor = new TalonFX(3);
  private final TalonFX RFSpeedMotor = new TalonFX(4);
  private final CANCoder RFangleEncoder = new CANCoder(22);

  private final TalonFX LRAngleMotor = new TalonFX(5);
  private final TalonFX LRSpeedMotor = new TalonFX(6);
  private final CANCoder LRangleEncoder = new CANCoder(23);

  private final TalonFX RRAngleMotor = new TalonFX(9);
  private final TalonFX RRSpeedMotor = new TalonFX(8);
  private final CANCoder RRangleEncoder = new CANCoder(24);

  private Sensors sensors;

  //Initialize WheelDrive objects
  private WheelDrive LeftFront = new WheelDrive(LFAngleMotor, LFSpeedMotor, LFangleEncoder);
  private WheelDrive RightFront = new WheelDrive(RFAngleMotor, RFSpeedMotor, RFangleEncoder);
  private WheelDrive LeftRear = new WheelDrive(LRAngleMotor, LRSpeedMotor, LRangleEncoder);
  private WheelDrive RightRear = new WheelDrive(RRAngleMotor, RRSpeedMotor, RRangleEncoder);

  //Put together swerve module positions relative to the center of the robot.
  private Translation2d FrontLeftLocation = new Translation2d((Constants.A_LENGTH/2), (Constants.A_WIDTH/2));
  private Translation2d FrontRightLocation = new Translation2d((Constants.A_LENGTH/2), -(Constants.A_WIDTH/2));
  private Translation2d BackLeftLocation = new Translation2d(-(Constants.A_LENGTH/2), (Constants.A_WIDTH/2));
  private Translation2d BackRightLocation = new Translation2d(-(Constants.A_LENGTH/2), -(Constants.A_WIDTH/2));

  //Create a kinematics withe the swerve module positions
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    FrontLeftLocation, FrontRightLocation, BackLeftLocation, BackRightLocation
  );

  //Initialize a ChassisSpeeds object and start it with default values
  private ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, new Rotation2d());

  private PIDController rotationPID = new PIDController(5e-1, 0.0, 0.0);

  //Initialize a list of module states and assign the kinematic results to them
  private SwerveModuleState[]  moduleStates = kinematics.toSwerveModuleStates(speeds);

  //Assign module states to modules
  private SwerveModuleState frontLeft = moduleStates[0];
  private SwerveModuleState frontRight = moduleStates[1];
  private SwerveModuleState backLeft = moduleStates[2];
  private SwerveModuleState backRight = moduleStates[3];

  //Create initial odometry
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics,
  new Rotation2d(), new Pose2d(Constants.STARTING_Y, Constants.STARTING_X, new Rotation2d()));

  //Store odometry as a position on the field.
  private Pose2d Position = odometry.update(new Rotation2d(), frontLeft, frontRight, backLeft, backRight);

  //Create a configuration for trajectories.
  private CentripetalAccelerationConstraint CentAccel = new CentripetalAccelerationConstraint(Constants.DRIVE_MAX_CACCEL);
  private TrajectoryConfig trajectoryConfig = new TrajectoryConfig(Constants.DRIVE_MAX_VELOCITY, Constants.DRIVE_MAX_ACCEL).addConstraint(CentAccel);

  //Initialize slew limiters.
  private SlewRateLimiter speedLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.0);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(180.0);

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain(Sensors sensors) {
    this.sensors = sensors;
  }

  /**
   * This method will be called once every scheduler run.
   */
  @Override
  public void periodic() {
    //update odometry and shuffleboard ever scheduler run.
    updateShuffleboard();
  }

  /**
   * Method for field oriented drive using kinematics
   * @param pilot CspController of the pilot
   */
  public void drive (CspController pilot) {
    //Convert controller input to M/S and Rad/S
    double Speed = (pilot.getY(Hand.kLeft, Scaling.CUBED)) * Constants.DRIVE_MAX_VELOCITY;
    double Strafe = (pilot.getX(Hand.kLeft, Scaling.CUBED)) * Constants.DRIVE_MAX_VELOCITY;

    double Angle = pilot.getAngle(Hand.kRight);
    double Rotation = pilot.getX(Hand.kRight);

    boolean fineControl = pilot.getBumper(Hand.kLeft);
    boolean fieldRelative = !pilot.getBumper(Hand.kRight);

    Speed = speedLimiter.calculate((fineControl) ? Speed / 2.0 : Speed);
    Strafe = strafeLimiter.calculate((fineControl) ? Strafe / 2.0 : Strafe);
    Angle = rotLimiter.calculate(Angle);
    Rotation = rotLimiter.calculate(Rotation * 180.0) / 180.0;

    //Get a chassis speed and rotation from input.
    speeds = (fieldRelative) ? (ChassisSpeeds.fromFieldRelativeSpeeds(
      Speed, Strafe, Rotation, sensors.getRotation2d())) :
      (new ChassisSpeeds(Speed, Strafe, Rotation));

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
    Position = odometry.update(sensors.getRotation2d(), frontLeft, frontRight, backLeft, backRight);
  }

  /**
   * Publish value from the drivetrain to the Smart Dashboard.
   */
  private void updateShuffleboard() {
    SmartDashboard.putString("Odometry", Position.toString());

    SmartDashboard.putNumber("Left Front Absolute Angle", LeftFront.getAbsoluteAngle());
    SmartDashboard.putNumber("Right Front Absolute Angle", RightFront.getAbsoluteAngle());
    SmartDashboard.putNumber("Left Rear Absolute Angle", LeftRear.getAbsoluteAngle());
    SmartDashboard.putNumber("Right Rear Absolute Angle", RightRear.getAbsoluteAngle());

    SmartDashboard.putNumber("Left Front RPM", LeftFront.getRPM());
    SmartDashboard.putNumber("Right Front RPM", RightFront.getRPM());
    SmartDashboard.putNumber("Left Rear RPM", LeftRear.getRPM());
    SmartDashboard.putNumber("Right Rear RPM", RightRear.getRPM());

    SmartDashboard.putNumber("Left Front Drive temp", getFrontLeftDriveTemp());
    SmartDashboard.putNumber("Left Front Angle temp", getFrontLeftAngleTemp());
    SmartDashboard.putNumber("Right Front Drive temp", getFrontRightDriveTemp());
    SmartDashboard.putNumber("Right Front Angle temp", getFrontRightAngleTemp());
    SmartDashboard.putNumber("Left Rear Drive temp", getRearLeftDriveTemp());
    SmartDashboard.putNumber("Left Rear Angle temp", getRearLeftAngleTemp());
    SmartDashboard.putNumber("Right Rear Drive temp", getRearRightDriveTemp());
    SmartDashboard.putNumber("Right Rear Angle temp", getRearRightAngleTemp());
  }

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
    return speeds;
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
    return Position;
  }

  /**
   * Get the Trajectory config object.
   * @return TrajectoryConfig object.
   */
  public TrajectoryConfig getConfig() {
    return trajectoryConfig;
  }

  public double getFrontLeftDriveTemp() {
    return LFSpeedMotor.getTemperature();
  }
  public double getFrontLeftAngleTemp() {
    return LFAngleMotor.getTemperature();
  }
  public double getFrontRightDriveTemp() {
    return RFSpeedMotor.getTemperature();
  }
  public double getFrontRightAngleTemp() {
    return RFAngleMotor.getTemperature();
  }
  public double getRearLeftDriveTemp() {
    return LRSpeedMotor.getTemperature();
  }
  public double getRearLeftAngleTemp() {
    return LRAngleMotor.getTemperature();
  }
  public double getRearRightDriveTemp() {
    return RRSpeedMotor.getTemperature();
  }
  public double getRearRightAngleTemp() {
    return RRAngleMotor.getTemperature();
  }
}