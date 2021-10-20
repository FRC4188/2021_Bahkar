// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.wpilibj.Solenoid;

/** Add your docs here. */
public class DSolenoid {

    Solenoid sol1;
    Solenoid sol2;

    public DSolenoid(int id1, int id2) {
        sol1 = new Solenoid(id1);
        sol2 = new Solenoid(id2);
        set(false);
    }

    public void set(boolean out) {
        sol1.set(out);
        sol2.set(!out);
    }

    public void relax() {
        sol1.set(false);
        sol2.set(false);
    }

    public boolean get() {
        return sol1.get();
    }
}
