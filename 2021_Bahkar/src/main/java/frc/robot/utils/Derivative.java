package frc.robot.utils;

public class Derivative {
    private double lastTime = System.currentTimeMillis();
    private double lastVal;

    public Derivative(double start) {
        lastVal = start;
    }

    public double getRate(double value) {
        double time = System.currentTimeMillis();
        double rate = (value - lastVal) / ((System.currentTimeMillis() - lastTime) / 1000.0);

        lastTime = time;
        lastVal = value;

        return rate;
    }
}
