package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.math.LogisticController;

/**
 * Class encapsulating climber function.
 */
public class Climber extends SubsystemBase {

    private static Climber instance;

    public synchronized static Climber getInstance() {
        if (instance == null) instance = new Climber();
        return instance;
    }

    private DualMotor motors = new DualMotor(13, 41);

    private LogisticController controller = new LogisticController(
        Constants.climber.P,
        Constants.climber.D,
        Constants.climber.S
    );

    // pneumatics
    private Solenoid climberSolenoid = new Solenoid(1);// needs to change

    private Notifier shuffle;

    /**
     * Constructs a new Climber object and configures devices.
     */
    private Climber() {

        //motors.setInverted(false);

        // init
        setBrake();
        setRampRate();

        // reset devices
        resetEncoders();

        shuffle = new Notifier(() -> updateShuffleboard());
        shuffle.startPeriodic(0.2);

        engagePneuBrake(false);
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
        SmartDashboard.putNumber("Left climber height", getPosition());
        SmartDashboard.putNumber("Left climber velocity", getVelocity());
        SmartDashboard.putBoolean("Climber brake", !climberSolenoid.get());
    }

    /**
     * Sets both climber motors to a given percentage [-1.0, 1.0].
     */
    public void set(double percent) {
        //if ((percent >= 0.0 && getPosition() < Constants.climber.MAX_HEIGHT) ||
            //(percent <= 0.0 && getPosition() > 0.0))
         motors.set(percent);
    }

    public void setVelocity(double velocity) {
        motors.set(controller.calculate(velocity, getVelocity()));
    }

    /**
     * Fires the break pistons to stop the climber.
     */
    public void engagePneuBrake(boolean output) {
        climberSolenoid.set(output);
    }

    public boolean getPneuBrake() {
        return climberSolenoid.get();
    }

    /**
     * Sets Climber motors to brake mode.
     */
    public void setBrake() {
        motors.brake(true);
    }

    /**
     * Sets climber ramp rates.
     */
    public void setRampRate() {
        motors.setRampRate(0.0);
    }

    /**
     * Resets encoder values to 0 for both sides of Climber.
     */
    public void resetEncoders() {
        motors.reset();
    }

    /**
     * Returns left encoder position in feet.
     */
    public double getPosition() {
        return motors.getPosition() * Constants.climber.ENCODER_TO_REV;
    }

    /**
     * Returns the left climber velocity in rpm.
     */
    public double getVelocity() {
        return motors.getVelocity() * Constants.climber.ENCODER_TO_REV;
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
    public double getFalconTemp() {
        return motors.getFalconTemp();
    }

    /**
     * Returns right climber motor temperature in Celcius.
     */
    public double getNeoTemp() {
        return motors.getNeoTemp();
    }

}