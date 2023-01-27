// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int CANDLE = 0;
    public static final int INTAKE_LEFT_TALONFX = 103;// TODO:REALNUMBER
    public static final int INTAKE_RIGHT_TALONFX = 103;// TODO:REALNUMBER
    public static final boolean INTAKE_LEFT_INVERTED = false;
    public static final boolean INTAKE_RIGHT_INVERTED = false;

    public static final int WRIST_TALONFX = 20;
    public static final int SHOULDER_TALONFX =19;
    public static final boolean SHOULDER_INVERTED = false;
    public static final boolean WRIST_INVERTED = false; 
    public static final int WRIST_DUTYENCODER = 0;
    public static final int SHOULDER_DUTYENCODER = 9;
    public static final double WRIST_ENCODER_OFFSET = 0.0;
    public static final double SHOULDER_ENCODER_OFFSET = 0.0;
    public static final double SHOULDER_GEAR_RATIO = (5.0 / 1.0) * (5.0 / 1.0) * (4.0 / 1.0) * (15.0 / 54.0);
    public static final double WRIST_GEAR_RATIO = (5.0 / 1.0) * (5.0 / 1.0) * (5.0 / 1.0);
    public static final double TALONFX_ENCODER_TICKS = 2048;
}
