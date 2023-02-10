package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConfig;
import frc.twilight.tunables.TunableDouble;

public class Arm extends SubsystemBase {
  private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(ArmConfig.WRIST_DUTYENCODER);
  private DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(ArmConfig.SHOULDER_DUTYENCODER);

  private TalonFX wrist = new TalonFX(ArmConfig.WRIST_TALONFX);
  private TalonFX shoulder = new TalonFX(ArmConfig.SHOULDER_TALONFX);

  ShuffleboardTab arm = Shuffleboard.getTab("arm");
  GenericEntry shoulderSB = arm.add("shoulder angle", 0).getEntry();
  GenericEntry wristSB = arm.add("wrist angle", 0).getEntry();

  private SlewRateLimiter shoulderAccel = new SlewRateLimiter(2);
  private SlewRateLimiter wristAccel = new SlewRateLimiter(2);

  private final boolean tunableDoubleEnabled = true;

  private final TunableDouble wristP = new TunableDouble("wristP", 0.1, tunableDoubleEnabled);
  private final TunableDouble wristI = new TunableDouble("wristI", 0, tunableDoubleEnabled);
  private final TunableDouble wristD = new TunableDouble("wristD", 0.2, tunableDoubleEnabled);

  private double oldWristP = wristP.getValue();
  private double oldWristI = wristI.getValue();
  private double oldWristD = wristD.getValue();

  private final TunableDouble shoulderP = new TunableDouble("shoulderP", 0.1, tunableDoubleEnabled);
  private final TunableDouble shoulderI = new TunableDouble("shoulderI", 0, tunableDoubleEnabled);
  private final TunableDouble shoulderD = new TunableDouble("shoulderD", 0.2, tunableDoubleEnabled);

  private double oldShoulderP = shoulderP.getValue();
  private double oldShoulderI = shoulderI.getValue();
  private double oldShoulderD = shoulderD.getValue();

  private double lastWristAngle = 0;
  private double lastShoulderAngle = 0;

  /** Config Objects for motor controllers */
  TalonFXConfiguration wristConfig = new TalonFXConfiguration();

  TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();

  public void updatePID() {

    if (wristP.getValue() != oldWristP) {
      wrist.config_kP(0, wristP.getValue());
      oldWristP = wristP.getValue();
    }

    if (wristI.getValue() != oldWristI) {
      wrist.config_kI(0, wristI.getValue());
      oldWristI = wristI.getValue();
    }

    if (wristD.getValue() != oldWristD) {
      wrist.config_kD(0, wristD.getValue());
      oldWristD = wristD.getValue();
    }

    if (shoulderP.getValue() != oldShoulderP) {
      shoulder.config_kP(0, shoulderP.getValue());
      oldShoulderP = shoulderP.getValue();
    }

    if (shoulderI.getValue() != oldShoulderI) {
      shoulder.config_kI(0, shoulderI.getValue());
      oldShoulderI = shoulderI.getValue();
    }

    if (shoulderD.getValue() != oldShoulderD) {
      shoulder.config_kD(0, shoulderD.getValue());
      oldShoulderD = shoulderD.getValue();
    }
  }

  public Arm() {
    /* Motion Magic Configurations */
    wristConfig.motionAcceleration = 2000;
    wristConfig.motionCruiseVelocity = 2000;

    shoulderConfig.motionCruiseVelocity = 2000;
    shoulderConfig.motionAcceleration = 2000;

    wrist.configAllSettings(wristConfig);
    shoulder.configAllSettings(shoulderConfig);

    wrist.configVoltageCompSaturation(10);
    shoulder.configVoltageCompSaturation(10);

    wrist.setInverted(ArmConfig.WRIST_INVERTED);
    shoulder.setInverted(ArmConfig.SHOULDER_INVERTED);

    SupplyCurrentLimitConfiguration supplyConfig = new SupplyCurrentLimitConfiguration();
    supplyConfig.currentLimit = 33;
    supplyConfig.enable = true;
    shoulder.configSupplyCurrentLimit(supplyConfig);
    wrist.configSupplyCurrentLimit(supplyConfig);

    StatorCurrentLimitConfiguration config = new StatorCurrentLimitConfiguration();
    config.currentLimit = 15;
    config.enable = true;
    shoulder.configStatorCurrentLimit(config);
    wrist.configStatorCurrentLimit(config);

    shoulder.setNeutralMode(NeutralMode.Brake);
    wrist.setNeutralMode(NeutralMode.Brake);

    wrist.config_kP(0, wristP.getValue());
    wrist.config_kI(0, wristI.getValue());
    wrist.config_kD(0, wristD.getValue());

    shoulder.config_kP(0, shoulderP.getValue());
    shoulder.config_kI(0, shoulderI.getValue());
    shoulder.config_kD(0, shoulderD.getValue());

    double wristOffset = getWristPosition() * (ArmConfig.WRIST_GEAR_RATIO) * (ArmConfig.TALONFX_ENCODER_TICKS);
    wrist.setSelectedSensorPosition(wristOffset);
    System.out.println("wristOffset = " + wristOffset);
    System.out.println("wristPosition = " + getWristPosition());

    double shoulderOffset = getShoulderPosition() * (ArmConfig.SHOULDER_GEAR_RATIO) * (ArmConfig.TALONFX_ENCODER_TICKS);
    shoulder.setSelectedSensorPosition(shoulderOffset);

    wristAccel.reset(0);
    shoulderAccel.reset(0);

    shoulder.configForwardSoftLimitEnable(true);
    shoulder.configForwardSoftLimitThreshold(
        anglesToShoulderSensorPosition(ArmConfig.SHOULDER_FORWARD_LIMIT));
    shoulder.configReverseSoftLimitEnable(true);
    shoulder.configReverseSoftLimitThreshold(
        anglesToShoulderSensorPosition(ArmConfig.SHOULDER_REVERSE_LIMIT));

    wrist.configForwardSoftLimitEnable(true);
    wrist.configForwardSoftLimitThreshold(
        anglesToWristSensorPosition(ArmConfig.WRIST_FORWARD_LIMIT));
    wrist.configReverseSoftLimitEnable(true);
    wrist.configReverseSoftLimitThreshold(
        anglesToWristSensorPosition(ArmConfig.WRIST_REVERSE_LIMIT));
  }

  public void overrideSoftLimits(boolean enabled) {
    System.out.println("overrideSoftLimits " + enabled);
    shoulder.overrideSoftLimitsEnable(enabled);
    wrist.overrideSoftLimitsEnable(enabled);
  }

  public double anglesToWristSensorPosition(double angle) {
    double gearRatio = ArmConfig.WRIST_GEAR_RATIO;
    double posValue = ((angle / 360.0) * gearRatio) * ArmConfig.TALONFX_ENCODER_TICKS;

    return posValue;
  }

  public double anglesToShoulderSensorPosition(double angle) {
    double gearRatio = ArmConfig.SHOULDER_GEAR_RATIO;
    double posValue = ((angle / 360.0) * gearRatio) * ArmConfig.TALONFX_ENCODER_TICKS;

    return posValue;
  }

  /** Arm enum for arm stataes */
  public enum ArmStates {
    INTAKE,
    MID_CUBE_NODE,
    HIGH_CUBE_NODE,
    MID_CONE_NODE,
    HIGH_CONE_NODE,
    MANUAL;
  }

  private ArmStates state = ArmStates.MANUAL;

  /** arm states */
  public void setArmState(ArmStates newState) {
    switch (newState) {
      case INTAKE:
        setPosition(135, -45);
        break;
      case MID_CUBE_NODE:
        setPosition(90, 0);
        break;
      case HIGH_CUBE_NODE:
        setPosition(40, 55);
        break;
      case MID_CONE_NODE:
        setPosition(85, 0);
        break;
      case HIGH_CONE_NODE:
        setPosition(15, 55);
        break;
      case MANUAL:
    }
  }

  private double degreesPerSecondToEncoderTicks(double angle, double gearRatio) {
    double gfx = ((angle / 360.0) * gearRatio) * ArmConfig.TALONFX_ENCODER_TICKS * 1 / 10;
    return gfx;
  }

  private void setPosition(double shouldereAng, double wristAng) {
    setShoulderAngle(shouldereAng);
    setWristAngle(wristAng);
  }

  public void zeroShoulder() {
    shoulder.setSelectedSensorPosition(0);
  }

  public void zeroWrist() {
    wrist.setSelectedSensorPosition(0);
  }

  public void setWristPercentOutput(double value) {
    value = wristAccel.calculate(value);

    wrist.set(TalonFXControlMode.PercentOutput, value);
  }

  public void setShoulderPercentOutput(double value) {
    value = shoulderAccel.calculate(value);

    shoulder.set(TalonFXControlMode.PercentOutput, value);
  }

  public void setWristAngle(double angle) {
    lastWristAngle = angle;
    double posValue = anglesToWristSensorPosition(angle);
    shoulder.set(TalonFXControlMode.Position, posValue);
  }

  public void setShoulderAngle(double angle) {
    lastShoulderAngle = angle;
    double posValue = anglesToShoulderSensorPosition(angle);
    shoulder.set(TalonFXControlMode.Position, posValue);
  }

  public void changeShoulderAngle(double amount) {
    setShoulderAngle(lastShoulderAngle + amount);
  }

  public void changeWristAngle(double amount) {
    setWristAngle(lastWristAngle + amount);
  }

  public double getShoulderPosition() {
    return shoulderEncoder.getAbsolutePosition() - ArmConfig.SHOULDER_ENCODER_OFFSET;
  }

  public double getWristPosition() {
    return wristEncoder.getAbsolutePosition() - ArmConfig.WRIST_ENCODER_OFFSET;
  }

  @Override
  public void periodic() {
    shoulderSB.setDouble(getShoulderPosition());
    wristSB.setDouble(getWristPosition());
    updatePID();
  }

  public ArrayList<TalonFX> geTalonFXs() {

    ArrayList<TalonFX> musicList = new ArrayList<>();
    musicList.add(shoulder);
    musicList.add(wrist);

    return musicList;
  }
}
