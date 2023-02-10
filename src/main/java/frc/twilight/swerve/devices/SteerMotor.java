package frc.twilight.swerve.devices;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.twilight.swerve.config.ModuleConfig;
import frc.twilight.swerve.config.PIDconfig;
import frc.twilight.tunables.TunableDouble;

public class SteerMotor {
  private static final int TIMOUT_MS = 10;

  private final TalonFX motor;
  public static final double TICKS_PER_REVOLUTION = 2048;
  public static final double ANGLE_UNITS_PER_ROTATION = 360;

  private TunableDouble p = PIDconfig.DT_STEER_P;
  private TunableDouble i = PIDconfig.DT_STEER_I;
  private TunableDouble d = PIDconfig.DT_STEER_D;

  private double oldP = p.getValue();
  private double oldI = i.getValue();
  private double oldD = d.getValue();

  public SteerMotor(int canID) {
    motor = new TalonFX(canID);

    // Reset motor settings
    motor.configFactoryDefault();

    // Set up PID
    motor.config_kP(0, p.getValue(), TIMOUT_MS);
    motor.config_kI(0, i.getValue(), TIMOUT_MS);
    motor.config_kD(0, d.getValue(), TIMOUT_MS);
    motor.config_kF(0, 0, TIMOUT_MS);

    // Sets motor inversion
    motor.setInverted(ModuleConfig.DT_STEER_MOTOR_INVERTED);

    // Motor is neutral when within range
    motor.configNeutralDeadband(0.1, TIMOUT_MS);

    // Set motor to use internal encoder
    motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, TIMOUT_MS);

    // Seconds from neutral to full
    motor.configClosedloopRamp(0.1, TIMOUT_MS);

    // Set encoder to 0
    motor.setSelectedSensorPosition(0, 0, TIMOUT_MS);

    // Set max/min output
    motor.configPeakOutputForward(1, TIMOUT_MS);
    motor.configPeakOutputReverse(-1, TIMOUT_MS);

    // Set nominal output
    motor.configNominalOutputForward(0, TIMOUT_MS);
    motor.configNominalOutputReverse(0, TIMOUT_MS);

    // Set current limiting
    motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 35, 0.5));
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 35, 0.5));

    // Set voltage comp
    motor.configVoltageCompSaturation(10);
    motor.enableVoltageCompensation(true);
  }

  public void setAngle(double angle) {
    motor.set(ControlMode.Position, angle * TICKS_PER_REVOLUTION / ANGLE_UNITS_PER_ROTATION);

    // Update PID values if they have changed
    if (p.getValue() != oldP) {
      motor.config_kP(0, p.getValue(), TIMOUT_MS);
      oldP = p.getValue();
    }

    if (i.getValue() != oldI) {
      motor.config_kI(0, i.getValue(), TIMOUT_MS);
      oldI = i.getValue();
    }

    if (d.getValue() != oldD) {
      motor.config_kD(0, d.getValue(), TIMOUT_MS);
      oldD = d.getValue();
    }
  }

  public double getAngle() {
    return motor.getSelectedSensorPosition() * ANGLE_UNITS_PER_ROTATION / TICKS_PER_REVOLUTION;
  }

  public double getRPM() {
    return motor.getSelectedSensorVelocity() / TICKS_PER_REVOLUTION * 600;
  }

  public void setInverted(boolean inverted) {
    motor.setInverted(
        inverted ? ModuleConfig.DT_STEER_MOTOR_INVERTED : !ModuleConfig.DT_STEER_MOTOR_INVERTED);
  }

  public void setEncoderPosition(double angle) {
    motor.setSelectedSensorPosition(
        angle * TICKS_PER_REVOLUTION / ANGLE_UNITS_PER_ROTATION, 0, TIMOUT_MS);
  }
}
