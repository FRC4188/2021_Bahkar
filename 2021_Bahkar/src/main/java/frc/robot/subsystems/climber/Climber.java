package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
import frc.robot.utils.DSolenoid;

/** Class encapsulating climber function. */
public class Climber extends SubsystemBase {

  private static Climber instance;

  public static synchronized Climber getInstance() {
    if (instance == null) instance = new Climber();
    return instance;
  }

  // private DualMotor motors = new DualMotor(13, 12);

  // pneumatics
  private DSolenoid climberSolenoid = new DSolenoid(3, 4);

  private Notifier shuffle;

  /** Constructs a new Climber object and configures devices. */
  private Climber() {

    // motors.setInverted(false);

    // init
    setBrake();
    setRampRate();

    // reset devices
    resetEncoders();

    shuffle = new Notifier(() -> updateShuffleboard());
    shuffle.startPeriodic(0.2);

    climberSolenoid.set(false);
  }

  /** Runs every loop. */
  @Override
  public void periodic() {
    // updateShuffleboard();
  }

  /** Writes values to the Shuffleboard. */
  public void updateShuffleboard() {
    SmartDashboard.putNumber("Left climber height", getPosition());
    SmartDashboard.putNumber("Left climber velocity", getVelocity());
    SmartDashboard.putBoolean("Climber Solenoid", getPneuBrake());
  }

  /** Sets both climber motors to a given percentage [-1.0, 1.0]. */
  public void set(double percent) {
    // if ((percent >= 0.0 && getPosition() < Constants.climber.MAX_HEIGHT) ||
    // (percent <= 0.0 && getPosition() > 0.0))
    // motors.set(percent);
  }

  public void setVelocity(double velocity) {
    // motors.set(controller.calculate(velocity, getVelocity()));
  }

  /** Fires the break pistons to stop the climber. */
  public void engagePneuBrake(boolean output) {
    climberSolenoid.set(output);
  }

  public void relax() {
    climberSolenoid.relax();
  }

  public boolean getPneuBrake() {
    return climberSolenoid.get();
  }

  /** Sets Climber motors to brake mode. */
  public void setBrake() {
    // motors.brake(true);
  }

  /** Sets climber ramp rates. */
  public void setRampRate() {
    // motors.setRampRate(Constants.climber.RAMP_RATE);
  }

  /** Resets encoder values to 0 for both sides of Climber. */
  public void resetEncoders() {
    // motors.reset();
  }

  /** Returns left encoder position in feet. */
  public double getPosition() {
    return 0.0; // motors.getPositions()[0] * Constants.climber.ENCODER_TO_REV;
  }

  /** Returns the left climber velocity in rpm. */
  public double getVelocity() {
    return 0.0; // motors.getVelocities()[0] * Constants.climber.ENCODER_TO_REV;
  }

  /** Returns left climber motor temperature in Celcius. */
  public double[] getFalconTemps() {
    return null; // motors.getTemps();
  }

  // /**
  //  * Returns right climber motor temperature in Celcius.
  //  */
  // public double getNeoTemp() {
  //     return motors.getNeoTemp();
  // }

}
