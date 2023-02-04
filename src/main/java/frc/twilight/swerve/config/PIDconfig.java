package frc.twilight.swerve.config;

import frc.twilight.tunables.TunableDouble;

public class PIDconfig {
  public static final boolean SHUFFLEBOARD_VALUES_ENABLED = false;

  // PID values for the drive motor
  public static final TunableDouble DT_DRIVE_P =
      new TunableDouble("DT_DRIVE_P", 0.1, SHUFFLEBOARD_VALUES_ENABLED).setSpot(0, 0);
  public static final TunableDouble DT_DRIVE_I =
      new TunableDouble("DT_DRIVE_I", 0, SHUFFLEBOARD_VALUES_ENABLED).setSpot(1, 0);
  public static final TunableDouble DT_DRIVE_D =
      new TunableDouble("DT_DRIVE_D", 0.2, SHUFFLEBOARD_VALUES_ENABLED).setSpot(2, 0);
  public static final TunableDouble DT_DRIVE_F =
      new TunableDouble("DT_DRIVE_F", 0.052, SHUFFLEBOARD_VALUES_ENABLED).setSpot(3, 0);

  // PID values for the steer motor
  public static final TunableDouble DT_STEER_P =
      new TunableDouble("DT_STEER_P", 0.4, SHUFFLEBOARD_VALUES_ENABLED).setSpot(0, 1);
  public static final TunableDouble DT_STEER_I =
      new TunableDouble("DT_STEER_I", 0.0001, SHUFFLEBOARD_VALUES_ENABLED).setSpot(1, 1);
  public static final TunableDouble DT_STEER_D =
      new TunableDouble("DT_STEER_D", 1.274, SHUFFLEBOARD_VALUES_ENABLED).setSpot(2, 1);

  // Autonomous PID constants
  public static final TunableDouble DT_AUTO_P =
      new TunableDouble("DT_AUTO_P", 2, SHUFFLEBOARD_VALUES_ENABLED).setSpot(0, 2);
  public static final TunableDouble DT_AUTO_I =
      new TunableDouble("DT_AUTO_I", 0.01, SHUFFLEBOARD_VALUES_ENABLED).setSpot(1, 2);
  public static final TunableDouble DT_AUTO_D =
      new TunableDouble("DT_AUTO_D", 0.22, SHUFFLEBOARD_VALUES_ENABLED).setSpot(2, 2);

  // Gyro autonomous PID constants
  public static final TunableDouble DT_AUTO_ROT_P =
      new TunableDouble("DT_AUTO_ROT_P", 1.5, SHUFFLEBOARD_VALUES_ENABLED).setSpot(0, 3);
  public static final TunableDouble DT_AUTO_ROT_I =
      new TunableDouble("DT_AUTO_ROT_I", 0.3, SHUFFLEBOARD_VALUES_ENABLED).setSpot(1, 3);
  public static final TunableDouble DT_AUTO_ROT_D =
      new TunableDouble("DT_AUTO_ROT_D", 0, SHUFFLEBOARD_VALUES_ENABLED).setSpot(2, 3);

  // Gyro velocity PID constants
  public static final TunableDouble DT_GYRO_P =
      new TunableDouble("DT_GYRO_P", 0, SHUFFLEBOARD_VALUES_ENABLED).setSpot(0, 4);
  public static final TunableDouble DT_GYRO_I =
      new TunableDouble("DT_GYRO_I", 0, SHUFFLEBOARD_VALUES_ENABLED).setSpot(1, 4);
  public static final TunableDouble DT_GYRO_D =
      new TunableDouble("DT_GYRO_D", 0, SHUFFLEBOARD_VALUES_ENABLED).setSpot(2, 4);
  public static final TunableDouble DT_GYRO_F =
      new TunableDouble("DT_GYRO_F", 0.01, SHUFFLEBOARD_VALUES_ENABLED).setSpot(3, 4);
}
