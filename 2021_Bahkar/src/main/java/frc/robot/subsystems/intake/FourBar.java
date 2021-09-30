package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Solenoid;

public class FourBar {

    private Solenoid masterSolenoid;

    private boolean raised = true;

    public FourBar(int masterID) {
        masterSolenoid = new Solenoid(masterID);

        setRaised(true);
    }

    public void setRaised(boolean raised) {
        this.raised = raised;

        masterSolenoid.set(!raised);
    }

    public boolean getRaised() {
        return raised;
    }
}