package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class CSPJoystick extends Joystick {

    private final int TRIGGER = 1;

    public CSPJoystick(int port) {
        super(port);
    }

    public double getXAxis() {
        return Math.abs(getRawAxis(0)) < 0.05 ? 0.0 : getRawAxis(0);
    }

    public double getYAxis() {
        return Math.abs(getRawAxis(1)) < 0.05 ? 0.0 : -getRawAxis(1);
    }

    public double getRotation() {
        return Math.abs(getRawAxis(3)) < 0.05 ? 0.0 : Math.pow(getRawAxis(3), 3.0);
    }

    public JoystickButton getTriggerButtonObj() {
        return new JoystickButton(this, TRIGGER);
    }

    public JoystickButton get2ButtonObj() {
        return new JoystickButton(this, 2);
    }

    public JoystickButton get3ButtonObj() {
        return new JoystickButton(this, 3);
    }

    public JoystickButton get4ButtonObj() {
        return new JoystickButton(this, 4);
    }

    public JoystickButton get5ButtonObj() {
        return new JoystickButton(this, 5);
    }

    public JoystickButton get6ButtonObj() {
        return new JoystickButton(this, 6);
    }
}