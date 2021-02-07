package frc.robot.utils.components;

import edu.wpi.first.wpilibj.Servo;

public class DualServos {
    Servo master, slave;
    public DualServos(Servo master, Servo slave) {
        this.master = master;
        this.slave = slave;
    }

    public void setBounds(double max, double deadbandMax, double center, double deadbandMin, double min) {
        master.setBounds(max, deadbandMax, center, deadbandMin, min);
        slave.setBounds(max, deadbandMax, center, deadbandMin, min);
    }

    public void setPos(double pos) {
        master.setPosition(pos);
        slave.setPosition(pos);
    }

    public void setSpeed(double speed) {
        master.setSpeed(speed);
        slave.setSpeed(speed);
    }

    public double getPos() {
        return master.getPosition();
    }
}