package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Class encapsulating climber function.
 */
public class Climber extends SubsystemBase {

    // motor init
    private WPI_TalonFX climberLeftMotor = new WPI_TalonFX(13);
    private WPI_TalonFX climberRightMotor = new WPI_TalonFX(12);

    // pneumatics
    private Solenoid climberSolenoid = new Solenoid(1);// needs to change
    boolean isBrakeEngaged;

    private Notifier shuffle;

    /**
     * Constructs a new Climber object and configures devices.
     */
    public Climber() {

        // setup encoders and inversions
        climberLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        climberRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
        climberLeftMotor.setInverted(false);
        climberRightMotor.setInverted(true);

        // init
        controllerInit();
        setBrake();
        setRampRate();

        // reset devices
        resetEncoders();

        shuffle = new Notifier(() -> updateShuffleboard());
        shuffle.startPeriodic(0.1);

        engagePneuBrake(true);
    }

    /**
     * Runs every loop.
     */
    @Override
    public void periodic() {
        //updateShuffleboard();
    }

    /**
     * Writes values to the Shuffleboard.
     */
    public void updateShuffleboard() {
        SmartDashboard.putNumber("Left climber height", getLeftPosition());
        SmartDashboard.putNumber("Right climber height", getRightPosition());
        SmartDashboard.putNumber("Left climber velocity", getLeftVelocity());
        SmartDashboard.putNumber("Right climber velocity", getRightVelocity());
        SmartDashboard.putBoolean("Climber brake", !climberSolenoid.get());
        SmartDashboard.putNumber("Climber Power", climberLeftMotor.get());
    }

    /**
     * Config Pid loop stuff. Have Locke explain.
     */
    public void controllerInit() {
        climberLeftMotor.config_kI(0, Constants.climber.kI, Constants.climber.TIMEOUT);
        climberLeftMotor.config_kD(0, Constants.climber.kD, Constants.climber.TIMEOUT);
        climberLeftMotor.config_kP(0, Constants.climber.kP, Constants.climber.TIMEOUT);
        climberLeftMotor.config_kF(0, Constants.climber.kF, Constants.climber.TIMEOUT);
        climberRightMotor.config_kI(0, Constants.climber.kI, Constants.climber.TIMEOUT);
        climberRightMotor.config_kD(0, Constants.climber.kD, Constants.climber.TIMEOUT);
        climberRightMotor.config_kP(0, Constants.climber.kP, Constants.climber.TIMEOUT);
        climberRightMotor.config_kF(0, Constants.climber.kF, Constants.climber.TIMEOUT);
    }

    /**
     * Sets both climber motors to a given percentage [-1.0, 1.0].
     */
    public void set(double percent) {
        setLeft(percent);
        setRight(percent);
    }

    /**
     * Sets left climber motor to a given percentage [-1.0, 1.0].
     */
    public void setLeft(double percent) {
        /**if (
            (getLeftPosition() <= Constants.climber.MAX_POSITION && percent < 0.0) ||
            (getLeftPosition() >= Constants.climber.MIN_POSITION && percent > 0.0)) percent = 0.0;*/
        climberLeftMotor.set(ControlMode.PercentOutput, percent);
    }

    /**
     * Sets left climber motor to a given percentage [-1.0, 1.0].
     */
    public void setRight(double percent) {
        climberRightMotor.set(ControlMode.PercentOutput, percent);
    }

    /**
     * Sets both motors to run at a given percentage [-1.0, 1.0] of max velocity.
     */
    public void setVelocity(double percent) {
        double velocity = (percent * Constants.climber.MAX_VELOCITY) / (Constants.climber.ENCODER_TO_REV * 600); //(climberLeftMotor.getSelectedSensorPosition() >= Constants.climber.MAX_POSITION && climberRightMotor.getSelectedSensorPosition() >= Constants.climber.MAX_POSITION) && percent > 0.0 ? 0.0 : (percent * Constants.climber.MAX_VELOCITY) / (Constants.climber.ENCODER_TO_REV * 600);
        climberLeftMotor.set(ControlMode.Velocity, velocity);
        climberRightMotor.set(ControlMode.Velocity, velocity);
    }

    /**
     * Sets left motor to run at a given percentage [-1.0, 1.0] of max velocity.
     */
    public void setLeftVelocity(double percent) {
        double velocity = (percent * Constants.climber.MAX_VELOCITY) / (Constants.climber.ENCODER_TO_REV * 600);
        climberLeftMotor.set(ControlMode.Velocity, velocity);
    }

    /**
     * Sets right motor to run at a given percentage [-1.0, 1.0] of max velocity.
     */
    public void setRightVelocity(double percent) {
        double velocity = (percent * Constants.climber.MAX_VELOCITY) / (Constants.climber.ENCODER_TO_REV * 600);
        climberRightMotor.set(ControlMode.Velocity, velocity);
    }

    /**
     * Fires the break pistons to stop the climber.
     */
    public void engagePneuBrake(boolean output) {
        climberSolenoid.set(output);
    }

    public void togglePneuBrake() {
        climberSolenoid.set(!climberSolenoid.get());
    }

    /**
     * Sets Climber motors to brake mode.
     */
    public void setBrake() {
        climberLeftMotor.setNeutralMode(NeutralMode.Brake);
        climberRightMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Sets climber ramp rates.
     */
    public void setRampRate() {
        climberLeftMotor.configOpenloopRamp(Constants.climber.RAMP_RATE);
        climberLeftMotor.configClosedloopRamp(0);
        climberRightMotor.configOpenloopRamp(Constants.climber.RAMP_RATE);
        climberRightMotor.configClosedloopRamp(0);
    }

    /**
     * Resets encoder values to 0 for both sides of Climber.
     */
    public void resetEncoders() {
        climberLeftMotor.setSelectedSensorPosition(0);
        climberRightMotor.setSelectedSensorPosition(0);
    }

    /**
     * Returns left encoder position in feet.
     */
    public double getLeftPosition() {
        return climberLeftMotor.getSelectedSensorPosition();
    }

    /**
     * Returns right encoder position in feet.
     */
    public double getRightPosition() {
        return climberRightMotor.getSelectedSensorPosition();
    }

    /**
     * Returns the left climber velocity in rpm.
     */
    public double getLeftVelocity() {
        return climberLeftMotor.getSelectedSensorVelocity()
                * Constants.climber.ENCODER_TO_REV * 600;// sensor units for rpm
    }

    /**
     * Returns the right climber velocity in rpm.
     */
    public double getRightVelocity() {
        return climberRightMotor.getSelectedSensorVelocity()
                * Constants.climber.ENCODER_TO_REV * 600;// sensor units for rpm
    }

    /**
     * Returns the maximum position of the climber in raw encoder ticks.
     */
    public double getMaxPosition() {
        return Constants.climber.MAX_POSITION;
    }

    /**
     * Returns the minimum position of the climber in raw encoder ticks.
     */
    public double getMinPosition() {
        return Constants.climber.MIN_POSITION;
    }

    /**
     * Returns left climber motor temperature in Celcius.
     */
    public double getLeftTemp() {
        return climberLeftMotor.getTemperature();
    }

    /**
     * Returns right climber motor temperature in Celcius.
     */
    public double getRightTemp() {
        return climberRightMotor.getTemperature();
    }

}