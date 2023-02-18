package frc.twilight.swerve.devices;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import frc.twilight.swerve.config.GeneralConfig;

public class Gyro {
  private final AHRS ahrs;

  private double prevPos = 0;
  private double prevSpeed = 0;
  private double time = System.currentTimeMillis() / 1000.0;

  public Gyro() {
    ahrs = new AHRS(SPI.Port.kMXP);
    setPosition(GeneralConfig.DT_START_GYRO);
  }

  public double getAngle() {
    return ahrs.getAngle();
  }

  public double getAngleSpeed() {
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

  public void setPosition(double offset) {
    ahrs.reset();
    ahrs.setAngleAdjustment(offset);
  }

  public void zeroSensor() {
    ahrs.reset();
    ahrs.setAngleAdjustment(0);

    time = System.currentTimeMillis() / 1000.0;
    prevPos = 0;
  }

  public double getYRot() {
    return ahrs.getPitch();
  }
}
