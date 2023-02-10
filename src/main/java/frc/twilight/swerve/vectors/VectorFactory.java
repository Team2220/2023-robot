package frc.twilight.swerve.vectors;

import frc.twilight.swerve.config.GeneralConfig;

public class VectorFactory {
  // Output wheel vectors in order of FR, FL, BR, BL
  public static WheelVector[] wheelVectorsFromDriveVector(DriveVector in) {
    WheelVector[] out = new WheelVector[4];
    out[0] = new WheelVector(0, 0);
    out[1] = new WheelVector(0, 0);
    out[2] = new WheelVector(0, 0);
    out[3] = new WheelVector(0, 0);

    double fwd = in.getFwd();
    double str = in.getStr();
    double rcw = in.getRcw();

    double a = str - rcw * (GeneralConfig.DT_LENGTH / GeneralConfig.DT_DIAMETER);
    double b = str + rcw * (GeneralConfig.DT_LENGTH / GeneralConfig.DT_DIAMETER);
    double c = fwd - rcw * (GeneralConfig.DT_WIDTH / GeneralConfig.DT_DIAMETER);
    double d = fwd + rcw * (GeneralConfig.DT_WIDTH / GeneralConfig.DT_DIAMETER);

    // Front Right
    out[0].setVelocity(Math.sqrt(b * b + c * c));
    out[0].setAngle(Math.atan2(b, c) * 180 / Math.PI);

    // Front Left
    out[1].setVelocity(Math.sqrt(b * b + d * d));
    out[1].setAngle(Math.atan2(b, d) * 180 / Math.PI);

    // Back Right
    out[2].setVelocity(Math.sqrt(a * a + c * c));
    out[2].setAngle(Math.atan2(a, c) * 180 / Math.PI);

    // Back Left
    out[3].setVelocity(Math.sqrt(a * a + d * d));
    out[3].setAngle(Math.atan2(a, d) * 180 / Math.PI);

    return out;
  }

  public static DriveVector driveVectorFromWheelVectors(
      WheelVector FR, WheelVector FL, WheelVector BR, WheelVector BL) {
    double aBR = Math.sin(Math.toRadians(BR.getAngle())) * BR.getVelocity();
    double cBR = Math.cos(Math.toRadians(BR.getAngle())) * BR.getVelocity();

    double aBL = Math.sin(Math.toRadians(BL.getAngle())) * BL.getVelocity();
    double dBL = Math.cos(Math.toRadians(BL.getAngle())) * BL.getVelocity();

    double bFR = Math.sin(Math.toRadians(FR.getAngle())) * FR.getVelocity();
    double cFR = Math.cos(Math.toRadians(FR.getAngle())) * FR.getVelocity();

    double bFL = Math.sin(Math.toRadians(FL.getAngle())) * FL.getVelocity();
    double dFL = Math.cos(Math.toRadians(FL.getAngle())) * FL.getVelocity();

    double a = (aBR + aBL) / 2;
    double b = (bFR + bFL) / 2;
    double c = (cFR + cBR) / 2;
    double d = (dFL + dBL) / 2;

    double rot1 = (b - a) / (GeneralConfig.DT_LENGTH / GeneralConfig.DT_DIAMETER);
    double rot2 = (c - d) / (GeneralConfig.DT_WIDTH / GeneralConfig.DT_DIAMETER);
    double rot = (rot1 + rot2) / 2;

    double fwd1 = rot * (GeneralConfig.DT_LENGTH / GeneralConfig.DT_DIAMETER) + a;
    double fwd2 = -rot * (GeneralConfig.DT_LENGTH / GeneralConfig.DT_DIAMETER) + b;
    double fwd = (fwd1 + fwd2) / 2;

    double str1 = rot * (GeneralConfig.DT_WIDTH / GeneralConfig.DT_DIAMETER) + c;
    double str2 = -rot * (GeneralConfig.DT_WIDTH / GeneralConfig.DT_DIAMETER) + d;
    double str = (str1 + str2) / 2;

    return new DriveVector(fwd, str, rot);
  }
}
