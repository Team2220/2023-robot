package frc.twilight.swerve.config;

import frc.twilight.tunables.TunableDouble;

public final class GeneralConfig {
  // Length of the drivetrain, measured from middle of wheel to middle of wheel
  // Doesn't need to be super accurate, just as a ratio of len to width
  public static final double DT_LENGTH = 1;

  // Width of the drivetrain, again, measured from middle of wheel to middle of wheel
  // Doesn't need to be super accurate, just as a ratio of len to width
  public static final double DT_WIDTH = 1;

  public static final double DT_START_GYRO = 0;
  public static final double DT_START_X = 0;
  public static final double DT_START_Y = 0;

  public static final boolean SWERVE_SHUFFLEBOARD_ENABLED = true;

  // Velocity restrictions m/s
  public static final TunableDouble DT_MAX_VEL =
      new TunableDouble("DT_MAX_VEL", 3, SWERVE_SHUFFLEBOARD_ENABLED, "Swerve").setSpot(4, 0);

  // Acceleration restrictions m/s2
  public static final TunableDouble DT_MAX_ACCEL =
      new TunableDouble("DT_MAX_ACCEL", 10, SWERVE_SHUFFLEBOARD_ENABLED, "Swerve").setSpot(5, 0);

  // Rotational velocity restrictions deg/s
  public static final TunableDouble DT_MAX_ROT_VEL =
      new TunableDouble("DT_MAX_ROT_VEL", 180, SWERVE_SHUFFLEBOARD_ENABLED, "Swerve").setSpot(4, 1);

  // Rotational acceleration restrictions deg/s2
  public static final TunableDouble DT_MAX_ROT_ACCEL = 
      new TunableDouble("DT_MAX_ROT_ACCEL", 360, SWERVE_SHUFFLEBOARD_ENABLED, "Swerve").setSpot(5, 1);

  // Don't touch these plz ty
  public static final double DT_DIAMETER =
      Math.sqrt(Math.pow(DT_LENGTH, 2) + Math.pow(DT_WIDTH, 2));
}
