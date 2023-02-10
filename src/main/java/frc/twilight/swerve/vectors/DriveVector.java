package frc.twilight.swerve.vectors;

import frc.twilight.helpfulThings.SlewRateLimiter;
import frc.twilight.swerve.config.GeneralConfig;

public class DriveVector {
  // Forward/backward velocity m/s
  private double fwd;
  // Left/right velocity m/s
  private double str;
  // Rotational velocity deg/s
  private double rcw;

  private static SlewRateLimiter fwdLimiter =
      new SlewRateLimiter(GeneralConfig.DT_MAX_ACCEL.getValue());
  private static SlewRateLimiter strLimiter =
      new SlewRateLimiter(GeneralConfig.DT_MAX_ACCEL.getValue());
  private static SlewRateLimiter rcwLimiter =
      new SlewRateLimiter(GeneralConfig.DT_MAX_ROT_ACCEL.getValue());

  public DriveVector(double fwd, double str, double rcw) {
    this.fwd = fwd;
    this.str = str;
    this.rcw = rcw;
  }

  public double getFwd() {
    return fwd;
  }

  public double getStr() {
    return str;
  }

  public double getRcw() {
    return rcw;
  }

  public void setFwd(double fwd) {
    this.fwd = fwd;
  }

  public void setStr(double str) {
    this.str = str;
  }

  public void setRcw(double rcw) {
    this.rcw = rcw;
  }

  public DriveVector zeroDirection(double facing) {
    double fwdOut =
        (fwd * Math.cos(Math.toRadians(facing))) + (str * Math.sin(Math.toRadians(facing)));
    double strOut =
        (str * Math.cos(Math.toRadians(facing))) - (fwd * Math.sin(Math.toRadians(facing)));

    fwd = fwdOut;
    str = strOut;

    return this;
  }

  public DriveVector maxVel() {
    if (Math.abs(fwd) > GeneralConfig.DT_MAX_VEL.getValue()) {
      double ratio = GeneralConfig.DT_MAX_VEL.getValue() / Math.abs(fwd);

      fwd *= ratio;
      str *= ratio;
    }

    if (Math.abs(str) > GeneralConfig.DT_MAX_VEL.getValue()) {
      double ratio = GeneralConfig.DT_MAX_VEL.getValue() / Math.abs(str);

      fwd *= ratio;
      str *= ratio;
    }

    if (rcw > GeneralConfig.DT_MAX_ROT_VEL.getValue()) {
      rcw = GeneralConfig.DT_MAX_ROT_VEL.getValue();
    } else if (rcw < -GeneralConfig.DT_MAX_ROT_VEL.getValue()) {
      rcw = -GeneralConfig.DT_MAX_ROT_VEL.getValue();
    }

    return this;
  }

  public DriveVector maxVel(double maxVel, double maxRotVel) {
    if (Math.abs(fwd) > maxVel) {
      double ratio = maxVel / Math.abs(fwd);

      fwd *= ratio;
      str *= ratio;
    }

    if (Math.abs(str) > maxVel) {
      double ratio = maxVel / Math.abs(str);

      fwd *= ratio;
      str *= ratio;
    }

    if (rcw > maxRotVel) {
      rcw = maxRotVel;
    } else if (rcw < -maxRotVel) {
      rcw = -maxRotVel;
    }

    return this;
  }

  public DriveVector maxAccel() {
    fwdLimiter.setRateLimit(GeneralConfig.DT_MAX_ACCEL.getValue());
    strLimiter.setRateLimit(GeneralConfig.DT_MAX_ACCEL.getValue());
    rcwLimiter.setRateLimit(GeneralConfig.DT_MAX_ROT_ACCEL.getValue());

    fwd = fwdLimiter.calculate(fwd);
    str = strLimiter.calculate(str);
    rcw = rcwLimiter.calculate(rcw);

    return this;
  }

  public static void resetAccel() {
    fwdLimiter.reset(0);
    strLimiter.reset(0);
    rcwLimiter.reset(0);
  }
}
