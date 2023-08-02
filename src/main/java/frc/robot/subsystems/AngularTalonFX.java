package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.fasterxml.jackson.annotation.JacksonInject.Value;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConfig;
import frc.robot.commands.Arm.SetArmState;
import frc.twilight.tunables.TunableDouble;
import java.util.ArrayList;
import java.util.Map;

class AngularTalonFX {

  private DutyCycleEncoder talonEncoder;
  private TalonFX talonFX;

  private final TunableDouble talonP;
  private final TunableDouble talonI;
  private final TunableDouble talonD;
  private final TunableDouble cruiseVel;
  private final TunableDouble acel;

  private double gearRatio;
  private double remapLimit;
  private double encoderOffset;

  private double talonRef;

  private String name;

  private TalonFXConfiguration talonConfig = new TalonFXConfiguration();

  public AngularTalonFX(
    int dutyEncoder,
    int talonId,
    String name,
    boolean tunableDoubleEnabled,
    double gearRatio,
    boolean inverted,
    double remapLimit,
    double encoderOffset,
    double forwardLimit,
    double reverseLimit,
    double talonRef
  ) {
    this.gearRatio = gearRatio;
    this.remapLimit = remapLimit;
    this.encoderOffset = encoderOffset;
    this.name = name;
    this.talonRef = talonRef;

    talonEncoder = new DutyCycleEncoder(dutyEncoder);
    talonFX = new TalonFX(talonId);

    talonP = new TunableDouble(name + "P", 0.1, tunableDoubleEnabled);
    talonI = new TunableDouble(name + "I", 0, tunableDoubleEnabled);
    talonD = new TunableDouble(name + "D", 0.2, tunableDoubleEnabled);

    acel = new TunableDouble(name + "Acel", 200, tunableDoubleEnabled);
    cruiseVel =
      new TunableDouble(name + "CruiseVel", 200, tunableDoubleEnabled);

    acel.addChangeListener(value -> {
      talonFX.configMotionAcceleration(degreesPerSecondToEncoderTicks(value));
    });
    cruiseVel.addChangeListener(value -> {
      talonFX.configMotionCruiseVelocity(degreesPerSecondToEncoderTicks(value));
    });

    talonP.addChangeListener(value -> {
      talonFX.config_kP(0, value);
    });
    talonI.addChangeListener(value -> {
      talonFX.config_kI(0, value);
    });
    talonD.addChangeListener(value -> {
      talonFX.config_kD(0, value);
    });

    talonConfig.motionAcceleration = degreesPerSecondToEncoderTicks(200);
    talonConfig.motionCruiseVelocity = degreesPerSecondToEncoderTicks(200);

    talonFX.configAllSettings(talonConfig);
    talonFX.configVoltageCompSaturation(10);
    talonFX.setInverted(inverted);

    SupplyCurrentLimitConfiguration supplyConfig = new SupplyCurrentLimitConfiguration();
    supplyConfig.currentLimit = 20;
    supplyConfig.enable = true;
    talonFX.configSupplyCurrentLimit(supplyConfig);

    StatorCurrentLimitConfiguration statorConfig = new StatorCurrentLimitConfiguration();
    statorConfig.currentLimit = 33;
    statorConfig.enable = true;
    talonFX.configStatorCurrentLimit(statorConfig);

    talonFX.setNeutralMode(NeutralMode.Brake);

    talonFX.config_kP(0, talonP.getValue());
    talonFX.config_kI(0, talonI.getValue());
    talonFX.config_kD(0, talonD.getValue());

    setTalonFromAbsEncoder();

    talonFX.configForwardSoftLimitEnable(true);
    talonFX.configForwardSoftLimitThreshold(
      anglesToTalonSensorPosition(forwardLimit)
    );
    talonFX.configReverseSoftLimitEnable(true);
    talonFX.configReverseSoftLimitThreshold(
      anglesToTalonSensorPosition(reverseLimit)
    );
  }

  private double degreesPerSecondToEncoderTicks(double angle) {
    double gfx =
      ((angle / 360.0) * gearRatio) *
      ArmConfig.TALONFX_ENCODER_TICKS *
      1.0 /
      10.0;
    return gfx;
  }

  public void setTalonFromAbsEncoder() {
    double talonOffset =
      getTalonPosition() * (gearRatio) * (ArmConfig.TALONFX_ENCODER_TICKS);
    talonFX.setSelectedSensorPosition(talonOffset);
  }

  public double getTalonPosition() {
    return (
      remap(talonEncoder.getAbsolutePosition(), remapLimit) - encoderOffset
    );
  }

  public double remap(double value, double limit) {
    if (value >= 0 && value < limit) {
      return value + 1;
    } else {
      return value;
    }
  }

  public double anglesToTalonSensorPosition(double angle) {
    double posValue =
      ((angle / 360.0) * gearRatio) * ArmConfig.TALONFX_ENCODER_TICKS;

    return posValue;
  }

  public double ticksToTalonAngle(double ticks) {
    double value = ticks / ArmConfig.TALONFX_ENCODER_TICKS / gearRatio;
    value *= 360;
    return value;
  }

  private double degreesPerSecondToEncoderTicks(
    double angle,
    double gearRatio
  ) {
    double gfx =
      ((angle / 360.0) * gearRatio) *
      ArmConfig.TALONFX_ENCODER_TICKS *
      1.0 /
      10.0;
    return gfx;
  }

  private void setPosition(double talonAng) {
    setTalonAngle(talonAng);
  }

  public void setTalonToReferenceAngle() {
    talonFX.setSelectedSensorPosition(anglesToTalonSensorPosition(talonRef));
  }

  public void setTalonPercentOutput(double value) {
    talonFX.set(TalonFXControlMode.PercentOutput, value);
  }

  public void setTalonAngle(double angle) {
    double posValue = anglesToTalonSensorPosition(-angle);
    talonFX.set(TalonFXControlMode.MotionMagic, posValue);
  }

  
}
