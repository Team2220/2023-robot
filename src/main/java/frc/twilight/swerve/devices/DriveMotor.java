package frc.twilight.swerve.devices;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.twilight.LogPowerFaults;
import frc.twilight.swerve.config.*;
import frc.twilight.tunables.TunableDouble;

public class DriveMotor {
  private static final int TIMOUT_MS = 10;

  private final TalonFX motor;
  private static final double TICKS_PER_REVOLUTION = 2048;

  private TunableDouble p = PIDconfig.DT_DRIVE_P;
  private TunableDouble i = PIDconfig.DT_DRIVE_I;
  private TunableDouble d = PIDconfig.DT_DRIVE_D;
  private TunableDouble f = PIDconfig.DT_DRIVE_F;

  private double oldP = p.getValue();
  private double oldI = i.getValue();
  private double oldD = d.getValue();
  private double oldF = f.getValue();

  public DriveMotor(int id) {
    motor = new TalonFX(id);

    // Reset motor settings
    motor.configFactoryDefault();

    LogPowerFaults.add(motor);

    // Set up PID
    motor.config_kP(0, p.getValue(), TIMOUT_MS);
    motor.config_kI(0, i.getValue(), TIMOUT_MS);
    motor.config_kD(0, d.getValue(), TIMOUT_MS);
    motor.config_kF(0, f.getValue(), TIMOUT_MS);

    // Sets motor inversion
    motor.setInverted(ModuleConfig.DT_DRIVE_MOTOR_INVERTED);

    // Motor is neutral when within range
    motor.configNeutralDeadband(0.001, TIMOUT_MS);

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
    motor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 60, 60, 0.5));
    motor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 60, 60, 0.5));

    // Set voltage comp
    motor.configVoltageCompSaturation(10);
    motor.enableVoltageCompensation(true);

    motor.setNeutralMode(NeutralMode.Brake);
  }

  public void setRPM(double rpm) {
    motor.set(ControlMode.Velocity, rpm * TICKS_PER_REVOLUTION / (60 * 10));

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

    if (f.getValue() != oldF) {
      motor.config_kF(0, f.getValue(), TIMOUT_MS);
      oldF = f.getValue();
    }
  }

  public double getRPM() {
    return motor.getSelectedSensorVelocity() * 600 / TICKS_PER_REVOLUTION;
  }

  public void getPosition() {
    motor.getSelectedSensorPosition();
  }

  public void setInverted(boolean inverted) {
    motor.setInverted(
        inverted ? ModuleConfig.DT_DRIVE_MOTOR_INVERTED : !ModuleConfig.DT_DRIVE_MOTOR_INVERTED);
  }
}
