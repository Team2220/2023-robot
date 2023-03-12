package frc.twilight.swerve.devices;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Gyro {
  private static final AHRS ahrs = new AHRS(SPI.Port.kMXP);

  private static double prevPos = 0;
  private static double prevSpeed = 0;
  private static double time = System.currentTimeMillis() / 1000.0;

  public static double getAngle() {
    return -ahrs.getAngle();
  }

  public static double getAngleSpeed() {
    double newTime = System.currentTimeMillis() / 1000.0;

    if ((newTime - time > 0.1)) {
      double newPos = getAngle();
      double speed = (newPos - prevPos) / (newTime - time);
      prevPos = newPos;
      time = newTime;
      prevSpeed = speed;

      return speed;
    } else {
      return prevSpeed;
    }
  }

  public static void setPosition(double offset) {
    ahrs.reset();
    ahrs.setAngleAdjustment(-offset);
  }

  public static void zeroSensor() {
    ahrs.reset();
    ahrs.setAngleAdjustment(0);

    time = System.currentTimeMillis() / 1000.0;
    prevPos = 0;
  }

  public static double getYRot() {
    return ahrs.getPitch();
  }
}
