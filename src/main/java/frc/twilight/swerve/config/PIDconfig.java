package frc.twilight.swerve.config;

import frc.twilight.tunables.TunableDouble;

public class PIDconfig {
    public static final boolean SHUFFLEBOARD_VALUES_ENABLED = false;


    // PID values for the drive motor
    public static final TunableDouble DT_DRIVE_P = new TunableDouble("DT_DRIVE_P", 0.1, SHUFFLEBOARD_VALUES_ENABLED);
    public static final TunableDouble DT_DRIVE_I = new TunableDouble("DT_DRIVE_I", 0, SHUFFLEBOARD_VALUES_ENABLED);
    public static final TunableDouble DT_DRIVE_D = new TunableDouble("DT_DRIVE_D", 0.2, SHUFFLEBOARD_VALUES_ENABLED);
    public static final TunableDouble DT_DRIVE_F = new TunableDouble("DT_DRIVE_F", 0.052, SHUFFLEBOARD_VALUES_ENABLED);



    // PID values for the steer motor
    public static final TunableDouble DT_STEER_P = new TunableDouble("DT_STEER_P", 0.1, SHUFFLEBOARD_VALUES_ENABLED);
    public static final TunableDouble DT_STEER_I = new TunableDouble("DT_STEER_I", 0, SHUFFLEBOARD_VALUES_ENABLED);
    public static final TunableDouble DT_STEER_D = new TunableDouble("DT_STEER_D", 0.2, SHUFFLEBOARD_VALUES_ENABLED);
    public static final TunableDouble DT_STEER_F = new TunableDouble("DT_STEER_F", 0, SHUFFLEBOARD_VALUES_ENABLED);



    // Autonomous PID constants
    public static final TunableDouble DT_AUTO_P = new TunableDouble("DT_AUTO_P", 0, SHUFFLEBOARD_VALUES_ENABLED);
    public static final TunableDouble DT_AUTO_I = new TunableDouble("DT_AUTO_I", 0, SHUFFLEBOARD_VALUES_ENABLED);
    public static final TunableDouble DT_AUTO_D = new TunableDouble("DT_AUTO_D", 0, SHUFFLEBOARD_VALUES_ENABLED);
    public static final TunableDouble DT_AUTO_F = new TunableDouble("DT_AUTO_F", 0, SHUFFLEBOARD_VALUES_ENABLED);

    
    
    // Gyro stabilize PID constants
    public static final TunableDouble DT_GYRO_P = new TunableDouble("DT_GYRO_P", 0, SHUFFLEBOARD_VALUES_ENABLED);
    public static final TunableDouble DT_GYRO_I = new TunableDouble("DT_GYRO_I", 0, SHUFFLEBOARD_VALUES_ENABLED);
    public static final TunableDouble DT_GYRO_D = new TunableDouble("DT_GYRO_D", 0, SHUFFLEBOARD_VALUES_ENABLED);
    public static final TunableDouble DT_GYRO_F = new TunableDouble("DT_GYRO_F", 0, SHUFFLEBOARD_VALUES_ENABLED);
}
