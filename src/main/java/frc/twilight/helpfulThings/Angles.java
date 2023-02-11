package frc.twilight.helpfulThings;

import frc.twilight.swerve.vectors.WheelVector;

public class Angles {
  public static double offsetAngle(double angle, double offset) {
    double out = angle - offset;

    // out = out % 360;

    // if (out < 0)
    //     out += 360;

    return out;
  }

  public static WheelVector optimizeWheel(WheelVector current, WheelVector set) {
    double angle = current.getAngle();
    double angleNew = set.getAngle();

    angleNew = flipAround(angle, angleNew);

    double velNew = set.getVelocity();

    double change = angle - angleNew;

    if (Math.abs(change) > 90) {
      angleNew += 180;
      velNew = -velNew;
    }

    return new WheelVector(velNew, angleNew);
  }

  public static double flipAround(double angle, double angleNew) {
    int angleRevo = (int) (angle / 360);

    angle %= 360;
    angleNew %= 360;

    double change = angle - angleNew;

    if (change > 180) angleNew += 360;
    else if (change < -180) angleNew -= 360;

    angleNew += 360 * angleRevo;

    return angleNew;
  }
}
