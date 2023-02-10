package frc.twilight.swerve;

import frc.twilight.helpfulThings.Angles;
import frc.twilight.swerve.config.ModuleConfig;
import frc.twilight.swerve.devices.DriveMotor;
import frc.twilight.swerve.devices.PWMencoder;
import frc.twilight.swerve.devices.SteerMotor;
import frc.twilight.swerve.vectors.WheelVector;

public class SwerveModule {
  private final SteerMotor steerMotor;
  private final DriveMotor driveMotor;
  public final PWMencoder steerEncoder;

  private final double steerOffset;

  private final double angleRatio = ModuleConfig.DT_STEER_GEAR_RATIO;
  private final double velocityRatio =
      (ModuleConfig.DT_DRIVE_GEAR_RATIO / 1) // Reduction from motor to output
          * (1 / (ModuleConfig.DT_WHEEL_DIAMETER * Math.PI)) // Convert from rotations to meters
          * (60 / 1); // Convert from minutes to seconds

  public SwerveModule(
      int driveMotorCanID, int steerMotorCanID, int steerEncoderAnalogPort, double steerOffset) {
    steerMotor = new SteerMotor(steerMotorCanID);
    driveMotor = new DriveMotor(driveMotorCanID);
    steerEncoder = new PWMencoder(steerEncoderAnalogPort);

    this.steerOffset = Angles.offsetAngle(steerEncoder.getPosition(), steerOffset);

    steerMotor.setEncoderPosition(-this.steerOffset * angleRatio);
  }

  public void set(WheelVector set) {
    set = Angles.optimizeWheel(get(), set);

    double angle = set.getAngle() * angleRatio;
    steerMotor.setAngle(angle);

    double speed = set.getVelocity() * velocityRatio;
    driveMotor.setRPM(speed);
  }

  public WheelVector get() {
    WheelVector out = new WheelVector(0, 0);
    double speed = driveMotor.getRPM() / velocityRatio;
    out.setVelocity(speed);

    double angle = steerMotor.getAngle() / angleRatio;
    out.setAngle(angle);

    return out;
  }

  public double getEncoderAngle() {
    return steerEncoder.getPosition();
  }
}
