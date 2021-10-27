package frc.robot.math;

import java.util.ArrayList;

public class Derivative {
  private double lastTime = System.currentTimeMillis();
  private double lastVal;

  public Derivative(double start) {
    lastVal = start;
  }

  public double getRate(double ds) {
    double time = System.currentTimeMillis();
    double rate = (ds - lastVal) / ((time - lastTime) / 1000.0);

    lastTime = time;
    lastVal = ds;

    return rate;
  }

  public double[] getRates(double[] values) {
    ArrayList<Double> ratesList = new ArrayList<Double>();
    for (double value : values) {
      ratesList.add(getRate(value));
    }
    return ratesList.stream().mapToDouble(d -> d).toArray();
  }
}
