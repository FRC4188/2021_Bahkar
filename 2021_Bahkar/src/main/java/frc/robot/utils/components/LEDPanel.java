package frc.robot.utils.components;

import edu.wpi.first.wpilibj.DigitalOutput;

public class LEDPanel {

    private DigitalOutput[][] ledMatrix = new DigitalOutput[4][5];

    public enum SYSTEM {
        SHOOTER(0), HOPPER(1), ROW3(2), GENERAL(3);

        private int index;
        private SYSTEM(int index) {
            this.index = index;
        }
        public int getIndex() {
            return index;
        }
    }

    public enum BEHAVIOR {
        ON, OFF, BLINK;
    }

    /**
     * The panel of LEDs used for 
     * @param minChannel The channel for the LEDs to start sending output to; will extend from that channel through the next 19.
     */
    public LEDPanel(int minChannel) {
        for (int row = 0; row < 4; row++) {
            for (int column = 0; column < 5; column++) {
                //ledMatrix[row][column] = new DigitalOutput(minChannel + 4 * row + column);
                //ledMatrix[row][column].setPWMRate(5e3);
                //ledMatrix[row][column].set(false);
            }
        }


    }

    public void set(SYSTEM system, int column, BEHAVIOR behavior) {
        set(system.getIndex(), column, behavior);
    }

    public void set(SYSTEM system, int column, boolean on) {
        set(system.getIndex(), column, on ? BEHAVIOR.ON : BEHAVIOR.OFF);
    }

    public void set(int row, int column, boolean on) {
        set(row, column, on ? BEHAVIOR.ON : BEHAVIOR.OFF);
    }

    public void set(int row, int column, BEHAVIOR behavior) {
        switch (behavior) {
            case ON:
                //ledMatrix[row][column].enablePWM(1.0);
                break;
            case BLINK:
                //ledMatrix[row][column].enablePWM(0.5);
                break;
            case OFF:
                //ledMatrix[row][column].disablePWM();
                //ledMatrix[row][column].set(false);
                break;
            default:
                //ledMatrix[row][column].disablePWM();
                //ledMatrix[row][column].set(false);
                break;
        }
    }
}