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

    int angleRevo = (int) (angle / 360);

    angle %= 360;
    angleNew %= 360;

    double velNew = set.getVelocity();

    double change = angle - angleNew;

    if (Math.abs(change) > 90 && Math.abs(change) < 270) {
      angleNew += 180;
      velNew = -velNew;
    } else if (change > 270) {
      angleNew += 360;
    } else if (change < -270) {
      angleNew -= 360;
    }

    angleNew += 360 * angleRevo;

    return new WheelVector(velNew, angleNew);
  }
}
