package frc.robot.subsystems.intake;

import frc.robot.utils.DSolenoid;

public class FourBar {

    private DSolenoid masterSolenoid;

    private boolean raised = true;

    public FourBar(int masterID) {
        masterSolenoid = new DSolenoid(1, 2);

        masterSolenoid.set(false);
    }

    public void setRaised(boolean raised) {
        this.raised = raised;

        masterSolenoid.set(raised);
    }

    public void relax() {
        masterSolenoid.relax();
    }

    public boolean getRaised() {
        return raised;
    }
}