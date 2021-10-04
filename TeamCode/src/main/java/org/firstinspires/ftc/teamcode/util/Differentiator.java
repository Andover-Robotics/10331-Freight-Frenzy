package org.firstinspires.ftc.teamcode.util;

public class Differentiator {
  double lastX;
  long lastT;

  public Differentiator(double initX) {
    lastX = initX;
    lastT = System.currentTimeMillis();
  }

  public double update(double x) {
    long now = System.currentTimeMillis();
    double dx = x - lastX, dt = now - lastT;
    lastX = x;
    lastT = now;
    return dx / dt;
  }
}
