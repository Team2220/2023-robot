package frc.twilight.swerve.config;

public class ModuleConfig {
    // Wheel diameter in meters
    public static final double DT_WHEEL_DIAMETER = 0.10033;

    // Drive gear ratio
    public static final double DT_DRIVE_GEAR_RATIO = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
    // Drive motor inverted
    public static final boolean DT_DRIVE_MOTOR_INVERTED = false;

    // Steer gear ratio
    public static final double DT_STEER_GEAR_RATIO = 7.0 / 150.0;
    // Steer motor inverted
    public static final boolean DT_STEER_MOTOR_INVERTED = true;

    // Steer encoder gear ratio
    public static final double DT_STEER_ENCODER_GEAR_RATIO = 1;
    // Steer encoder inverted
    public static final boolean DT_STEER_ENCODER_INVERTED = false;

    // Steer CANcoder offset front left
    public static final double DT_FL_SE_OFFSET = 68.2910; 

    // Steer CANcoder offset front right
    public static final double DT_FR_SE_OFFSET = 279.4042; 

    // Steer CANcoder offset back left
    public static final double DT_BL_SE_OFFSET = 303.3984;

    // Steer CANcoder offset back right
    public static final double DT_BR_SE_OFFSET = 128.8476; 
}
