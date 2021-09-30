package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj.Servo;

public class LinearServos {

    Servo master;
    Servo slave;

    public LinearServos(int pwmID1, int pwmID2) {
        master = new Servo(pwmID1);
        slave = new Servo(pwmID2);

        master.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        slave.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    }

    public void set(double position) {
        master.set(position);
        slave.set(position);
    }
}