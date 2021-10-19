package frc.robot.subsystems.climber;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.math.Derivative;

/** Class to control the Falcon/Neo duo in the climber */
public class DualMotor {

    private WPI_TalonFX falcon1;
    //private CANSparkMax neo;
    private WPI_TalonFX falcon2;

    private Derivative accel = new Derivative(0.0);

    /**
     * Constructs a new DualMotor controller.
     * @param falconID CAN ID of the falcon motor in the assembly.
     * @param neoID CAN ID of the neo motor in the assembly.
     */
    public DualMotor(int falcon1ID, /*int neoID*/ int falcon2ID) {
        falcon1 = new WPI_TalonFX(falcon1ID);
        //neo = new CANSparkMax(neoID, MotorType.kBrushless);
        falcon2 = new WPI_TalonFX(falcon2ID);

        falcon1.setInverted(false);
        //neo.setInverted(false);
        falcon2.setInverted(false);

        falcon1.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        falcon2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    }
    
    /**
     * Set the power of each motor.
     * @param percent Power in range [-1.0, 1.0].
     */
    public void set(double percent) {
        falcon1.set(percent);
        //neo.set(percent);
        falcon2.set(percent);
    }

    /**
     * Control the motor brake modes.
     * @param engaged True if braking, false if coasting.
     */
    public void brake(boolean engaged) {
        falcon1.setNeutralMode(engaged ? NeutralMode.Brake : NeutralMode.Coast);
        falcon2.setNeutralMode(engaged ? NeutralMode.Brake : NeutralMode.Coast);
        //neo.setIdleMode(engaged ? IdleMode.kBrake : IdleMode.kCoast);
    }

    /**
     * Control the inversion of the motor.
     */
    public void setInverted(boolean inverted) {
        falcon1.setInverted(inverted);
        //neo.setInverted(inverted);
        falcon2.setInverted(inverted);
    }

    /**
     * Control the ramp rate of the motors.
     * @param rate Time to go from 0 to 1.0 power.
     */
    public void setRampRate(double rate) {
        falcon1.configOpenloopRamp(rate);
        //neo.setOpenLoopRampRate(rate);
        falcon2.configOpenloopRamp(rate);
    }

    /**
     * Reset the encoder of the falcon motor.
     */
    public void reset() {
        falcon1.setSelectedSensorPosition(0);
        falcon2.setSelectedSensorPosition(0);
    }

    /**
     * Get the position of the falcon motor.
     */
    public double[] getPositions() {
        double[] positions = {falcon1.getSelectedSensorPosition(), falcon2.getSelectedSensorPosition()};
        return positions;
    }

    /**
     * Get the velocity of the falcon motor.
     */
    public double[] getVelocities() {
        double[] velocities = {falcon1.getSelectedSensorVelocity(), falcon2.getSelectedSensorVelocity()};
        return velocities;
    }

    /**
     * Get the acceleration of the falcon motor.
     */
    public double[] getAcceleration() {
        return accel.getRates(getVelocities());
    }

    /**
     * Return the temperature of the falcon motor in degrees Celsius.
     */
    public double[] getTemps() {
        double[] temps = {falcon1.getTemperature(), falcon2.getTemperature()};
        return temps;
    }

    // /**
    //  * Return the temperature of the neo motor in degrees Celsius.
    //  */
    // public double getNeoTemp() {
    //     return neo.getMotorTemperature();
    // }
}
