/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.fasterxml.jackson.databind.ser.std.NumberSerializers.DoubleSerializer;

import edu.wpi.first.wpilibj.Notifier;
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
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.CSPMath;
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

  private final TalonFX RRAngleMotor = new TalonFX(7);
  private final TalonFX RRSpeedMotor = new TalonFX(8);
  private final CANCoder RRangleEncoder = new CANCoder(24);

  private Sensors sensors;

  //Initialize WheelDrive objects
  private WheelDrive LeftFront = new WheelDrive(LFAngleMotor, LFSpeedMotor, LFangleEncoder, 73.125, false, false);
  private WheelDrive RightFront = new WheelDrive(RFAngleMotor, RFSpeedMotor, RFangleEncoder, 172.7, true, false);
  private WheelDrive LeftRear = new WheelDrive(LRAngleMotor, LRSpeedMotor, LRangleEncoder, 4.3, false, true);
  private WheelDrive RightRear = new WheelDrive(RRAngleMotor, RRSpeedMotor, RRangleEncoder, 158.37, true, true);

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

  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, Rotation2d.fromDegrees(sensors.getFusedHeading()));
  }

  /**p
   * Publish value from the drivetrain to the Smart Dashboard.
   */
  private void updateShuffleboard() {
    SmartDashboard.putString("Odometry", odometry.getPoseMeters().toString());
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

  private void setAnglePID(double kP, double kI, double kD) {
    LeftFront.setAnglePID(kP, kI, kD);
    RightFront.setAnglePID(kP, kI, kD);
    LeftRear.setAnglePID(kP, kI, kD);
    RightRear.setAnglePID(kP, kI, kD);
  }

  private void setSpeedPID(double kP, double kI, double kD) {
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
    return odometry.getPoseMeters();
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