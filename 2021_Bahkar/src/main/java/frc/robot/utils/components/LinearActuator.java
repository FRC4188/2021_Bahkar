package frc.robot.utils.components;

import edu.wpi.first.wpilibj.Servo;

public class LinearActuator extends Servo{
    public LinearActuator(int channel) {
        super(channel);

        this.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    }
}