package frc.robot.utils;

import edu.wpi.first.wpilibj.Solenoid;

/** Class to control a double-solenoid. */
public class DSolenoid {

    /** Pushing solenoid. */
    private Solenoid sol1;
    /** Pulling solenoid. */
    private Solenoid sol2;

    /**
     * Creates a new DSolenoid object.
     * @param id1 The ID of the pushing solenoid.
     * @param id2 The ID of the pulling solenoid.
     */
    public DSolenoid(int id1, int id2) {
        sol1 = new Solenoid(id1);
        sol2 = new Solenoid(id2);
        set(false);
    }

    /**
     * Control the set position of the solenoid.
     * @param out True if deployed, false if stowed.
     */
    public void set(boolean out) {
        sol1.set(out);
        sol2.set(!out);
    }

    /**
     * Close both solenoids.
     */
    public void relax() {
        sol1.set(false);
        sol2.set(false);
    }

    /**
     * Gets the position of the solenoid system.
     * @return True if the solenoid system is deployed, false if it is stowed.
     */
    public boolean get() {
        return sol1.get();
    }
}
