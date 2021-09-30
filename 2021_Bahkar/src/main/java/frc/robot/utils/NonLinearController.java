package frc.robot.utils;

import frc.robot.math.Derivative;

public class NonLinearController {

    private double a;
    private double b;
    private double d;
    private Derivative derivative = new Derivative(0.0);

    /**
     * Constructs a new NonLinearController
     * @param a The logistic "stretch" coefficient.
     * @param b The parabolic "stretch" coefficient.
     * @param d The derivative coefficient.
     */
    public NonLinearController(double a, double b, double d) {
        this.a = a;
        this.b = b;
        this.d = d;
    }

    public double calculate(double set, double measure) {
        double error = set - measure;

        double logistic = 2.0 / (1.0 + Math.pow(Math.E, -(1.0 / a) * error)) - 1.0;
        double parabolic = (1.0 / b) * Math.pow(error, 2.0);

        double Kp = logistic * parabolic;
        Kp = Math.abs(Kp) < 1.0 ? Kp : 1.0 * Math.signum(Kp);
        double Kd = derivative.getRate(error) * d;

        return Kp + Kd;
    }
}