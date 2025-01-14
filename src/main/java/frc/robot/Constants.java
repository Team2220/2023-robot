// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double ROBOT_X_LENGTH = Units.inchesToMeters(31.768);
  public static final double ROBOT_Y_LENGTH = Units.inchesToMeters(31.768);

  public static final class LEDS {
    public static final int LEFT = 1;
    public static final int RIGHT = 2;
  }

  public static final class LimelightConfig {
    public static final String LEFT_NAME = "limelight-left";
    public static final String RIGHT_NAME = "limelight-right";
    public static final String LEFT_URL = "http://10.22.20.10:5800/stream.mjpg";
    public static final String RIGHT_URL = "http://10.22.20.11:5800/stream.mjpg";
  }

  public static final class IntakeConfig {
    public static final int INTAKE_TALONFX = 10;
    public static final boolean INTAKE_INVERTED = false;
  }

  public static final class ArmConfig {
    public static final int WRIST_TALONFX = 20;
    public static final int SHOULDER_TALONFX = 19;
    public static final boolean SHOULDER_INVERTED = false;
    public static final boolean WRIST_INVERTED = false;
    public static final int WRIST_DUTYENCODER = 0;
    public static final int SHOULDER_DUTYENCODER = 1;
    public static final double WRIST_ENCODER_OFFSET_DEGREES = 0.432833635820841 * 360.0;
    public static final double SHOULDER_ENCODER_OFFSET_DEGREES = 0.9632 * 360.0;
    public static final double SHOULDER_GEAR_RATIO = (5.0 / 1.0) * (5.0 / 1.0) * (4.0 / 1.0) * (58.0 / 15.0);
    public static final double WRIST_GEAR_RATIO = (5.0 / 1.0) * (5.0 / 1.0) * (5.0 / 1.0);
    public static final double WRIST_FORWARD_LIMIT = 135.0;
    public static final double WRIST_REVERSE_LIMIT = -160.0;
    public static final double SHOULDER_FORWARD_LIMIT = 165.0; // Away from battery
    public static final double SHOULDER_REVERSE_LIMIT = -100.0; // Towards battery
    // Stowed position
    public static final double SHOULDER_REF = 171;
    public static final double WRIST_REF = 150;

    public static final double WRIST_REMAP_LIMIT_DEGREES = 0;
    public static final double SHOULDER_REMAP_LIMIT_DEGREES = 180;
    public static final double TALONFX_ENCODER_TICKS = 0;
  }
}
