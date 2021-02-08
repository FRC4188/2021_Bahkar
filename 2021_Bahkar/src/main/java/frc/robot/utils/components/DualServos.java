package frc.robot.utils.components;

import edu.wpi.first.wpilibj.Servo;

/**
 * Drives two servos together (for the shooter hood).
 */
public class DualServos {

    // Create empty servo objects to contain servos later.
    Servo master, slave;

    /**
     * Constructs a DualServos object.
     * @param master One servo object.
     * @param slave The other servo object.
     */
    public DualServos(Servo master, Servo slave) {
        this.master = master;
        this.slave = slave;
    }

    /**
     * Initializing the servo objects' bounds.
     * @param max
     * @param deadbandMax
     * @param center
     * @param deadbandMin
     * @param min
     */
    public void setBounds(double max, double deadbandMax, double center, double deadbandMin, double min) {
        master.setBounds(max, deadbandMax, center, deadbandMin, min);
        slave.setBounds(max, deadbandMax, center, deadbandMin, min);
    }

    /**
     * Send the servos to a certain position.
     * @param pos Position in range [0.0, 1.0] to move servos to.
     */
    public void setPos(double pos) {
        master.setPosition(pos);
        slave.setPosition(pos);
    }

    /**
     * Set the speed of the servo.
     * @param speed Speed in range [-1.0, 1.0] for the servos to move at.
     */
    public void setSpeed(double speed) {
        master.setSpeed(speed);
        slave.setSpeed(speed);
    }

    /**
     * Return the position of the servo.
     * @return Position in range [0.0, 1.0] servos are set to..
     */
    public double getPos() {
        return master.getPosition();
    }
}