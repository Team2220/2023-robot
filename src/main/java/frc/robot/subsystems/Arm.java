package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

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

public class Arm extends SubsystemBase {
  private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(ArmConfig.WRIST_DUTYENCODER);
  private DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(ArmConfig.SHOULDER_DUTYENCODER);

  private TalonFX wrist = new TalonFX(ArmConfig.WRIST_TALONFX);
  private TalonFX shoulder = new TalonFX(ArmConfig.SHOULDER_TALONFX);

  ShuffleboardTab arm = Shuffleboard.getTab("arm");

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
    wristConfig.motionAcceleration = degreesPerSecondToEncoderTicks(10, ArmConfig.WRIST_GEAR_RATIO);
    wristConfig.motionCruiseVelocity =
        degreesPerSecondToEncoderTicks(10, ArmConfig.WRIST_GEAR_RATIO);

    shoulderConfig.motionCruiseVelocity =
        degreesPerSecondToEncoderTicks(10, ArmConfig.SHOULDER_GEAR_RATIO);
    shoulderConfig.motionAcceleration =
        degreesPerSecondToEncoderTicks(10, ArmConfig.SHOULDER_GEAR_RATIO);

    wrist.configAllSettings(wristConfig);
    shoulder.configAllSettings(shoulderConfig);

    wrist.configVoltageCompSaturation(10);
    shoulder.configVoltageCompSaturation(10);

    wrist.setInverted(ArmConfig.WRIST_INVERTED);
    shoulder.setInverted(ArmConfig.SHOULDER_INVERTED);

    SupplyCurrentLimitConfiguration supplyConfig = new SupplyCurrentLimitConfiguration();
    supplyConfig.currentLimit = 20;
    supplyConfig.enable = true;
    shoulder.configSupplyCurrentLimit(supplyConfig);
    wrist.configSupplyCurrentLimit(supplyConfig);

    StatorCurrentLimitConfiguration config = new StatorCurrentLimitConfiguration();
    config.currentLimit = 33;
    config.enable = true;
    shoulder.configStatorCurrentLimit(config);
    wrist.configStatorCurrentLimit(config);

    // TODO Fix to break
    shoulder.setNeutralMode(NeutralMode.Brake);
    wrist.setNeutralMode(NeutralMode.Brake);

    wrist.config_kP(0, wristP.getValue());
    wrist.config_kI(0, wristI.getValue());
    wrist.config_kD(0, wristD.getValue());

    shoulder.config_kP(0, shoulderP.getValue());
    shoulder.config_kI(0, shoulderI.getValue());
    shoulder.config_kD(0, shoulderD.getValue());

    setWristFromAbsEncoder();

    setShoulderFromAbsEncoder();

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

    setUpTestCommands();
  }

  private void setShoulderFromAbsEncoder() {
    double shoulderOffset =
        getShoulderPosition() * (ArmConfig.SHOULDER_GEAR_RATIO) * (ArmConfig.TALONFX_ENCODER_TICKS);
    shoulder.setSelectedSensorPosition(shoulderOffset);
  }

  private void setWristFromAbsEncoder() {
    double wristOffset =
        getWristPosition() * (ArmConfig.WRIST_GEAR_RATIO) * (ArmConfig.TALONFX_ENCODER_TICKS);
    wrist.setSelectedSensorPosition(wristOffset);
  }

  public void overrideWristSoftLimits(boolean enabled) {
    DataLogManager.log("overrideWristSoftLimits " + enabled);
    wrist.overrideSoftLimitsEnable(enabled);
  }

  public void overrideShoulderSoftLimits(boolean enabled) {
    DataLogManager.log("overrideSholderSoftLimits " + enabled);
    shoulder.overrideSoftLimitsEnable(enabled);
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

  public double ticksToWristAngle(double ticks) {
    double value = ticks / ArmConfig.TALONFX_ENCODER_TICKS / ArmConfig.WRIST_GEAR_RATIO;
    value *= 360;
    return value;
  }

  public double ticksToShoulderAngle(double ticks) {
    double value = ticks / ArmConfig.TALONFX_ENCODER_TICKS / ArmConfig.SHOULDER_GEAR_RATIO;
    value *= 360;
    return value;
  }

  /** Arm enum for arm stataes */
  public enum ArmStates {
    INTAKE,
    MID_CUBE_NODE,
    HIGH_CUBE_NODE,
    MID_CONE_NODE,
    HIGH_CONE_NODE,
    TRANSIT,
    // Starting: Shoulder = 171 , Wrist = 150
    SINGLE_LOADING_STATION,
    DOUBLE_LOADING_STATION
  }

  /** arm states */
  public void setArmState(ArmStates newState) {
    DataLogManager.log("Setting arm state to " + newState.name());
    switch (newState) {
      case INTAKE:
        setPosition(152, 41);
        break;
      case MID_CUBE_NODE:
        setPosition(60, -90);
        break;
      case HIGH_CUBE_NODE:
        setPosition(70, -40);
        break;
      case MID_CONE_NODE:
        setPosition(60, -85);
        break;
      case HIGH_CONE_NODE:
        setPosition(70, -20);
        break;
      case TRANSIT:
        setPosition(171, 150);
        break;
      case SINGLE_LOADING_STATION:
        setPosition(171, 140);
        break;
      case DOUBLE_LOADING_STATION:
        setPosition(68, -43);
        break;
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

  public void setShoulderToReferenceAngle() {
    shoulder.setSelectedSensorPosition(
        anglesToShoulderSensorPosition(Constants.ArmConfig.SHOULDER_REF));
  }

  public void setWristToReferenceAngle() {
    wrist.setSelectedSensorPosition(anglesToWristSensorPosition(Constants.ArmConfig.WRIST_REF));
  }

  public void setWristPercentOutput(double value) {
    wrist.set(TalonFXControlMode.PercentOutput, value);
  }

  public void setShoulderPercentOutput(double value) {
    shoulder.set(TalonFXControlMode.PercentOutput, value);
  }

  public void setWristAngle(double angle) {
    lastWristAngle = angle;
    double posValue = anglesToWristSensorPosition(angle);
    wrist.set(TalonFXControlMode.Position, posValue);
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
    return remap(shoulderEncoder.getAbsolutePosition(), ArmConfig.SHOULDER_REMAP_LIMIT) - ArmConfig.SHOULDER_ENCODER_OFFSET;
  }

  public double getWristPosition() {

    return remap(wristEncoder.getAbsolutePosition(), ArmConfig.WRIST_REMAP_LIMIT) - ArmConfig.WRIST_ENCODER_OFFSET;
  }

  public void setUpTestCommands() {
    // Arm States
    ShuffleboardLayout stateLayout = Shuffleboard.getTab("arm")
        .getLayout("States", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withProperties(Map.of("Label position", "HIDDEN"));

    for (ArmStates state : ArmStates.values()) {
      stateLayout.add(state.name(), new SetArmState(state, this));
    }

    // Test Positions
    ShuffleboardLayout testPositionLayout = Shuffleboard.getTab("arm")
        .getLayout("Test Positions", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withProperties(Map.of("Label position", "HIDDEN"));

    testPositionLayout.add(
        "ZeroShoulder",
        new InstantCommand(() -> shoulder.setSelectedSensorPosition(0)).withName("ZeroShoulder"));
    testPositionLayout.add(
        "ZeroWrist",
        new InstantCommand(() -> wrist.setSelectedSensorPosition(0)).withName("ZeroWrist"));

    testPositionLayout.add(
        "setShoulderFromEncoder",
        new InstantCommand(() -> setShoulderFromAbsEncoder()).withName("setShoulderFromEncoder"));
    testPositionLayout.add(
        "setWristFromEncoder",
        new InstantCommand(() -> setWristFromAbsEncoder()).withName("setWristFromEncoder"));

    testPositionLayout.add(
        "ReferenceShoulder",
        new InstantCommand(() -> setShoulderToReferenceAngle()).withName("ReferenceShoulder"));
    testPositionLayout.add(
        "RefernceWrist",
        new InstantCommand(() -> setWristToReferenceAngle()).withName("RefernceWrist"));
    testPositionLayout.add(
        "Wrist=90", new InstantCommand(() -> setWristAngle(90)).withName("Wrist=90"));
    testPositionLayout.add(
        "Wrist=0", new InstantCommand(() -> setWristAngle(0)).withName("Wrist=0"));
    testPositionLayout.add(
        "Wrist=-90", new InstantCommand(() -> setWristAngle(-90)).withName("Wrist=-90"));
    testPositionLayout.add(
        "Shoulder=0", new InstantCommand(() -> setShoulderAngle(0)).withName("Shoulder=0"));
    testPositionLayout.add(
        "Shoulder=90", new InstantCommand(() -> setShoulderAngle(90)).withName("Shoulder=90"));
    testPositionLayout.add(
        "Shoulder=-90", new InstantCommand(() -> setShoulderAngle(-90)).withName("Shoulder=-90"));

    // Everything else
    ShuffleboardLayout angLayout = Shuffleboard.getTab("arm")
        .getLayout("Angles", BuiltInLayouts.kGrid)
        .withSize(3, 4)
        .withProperties(Map.of("Label position", "TOP"));

    angLayout.addDouble("shoulder raw abs encoder", shoulderEncoder::getAbsolutePosition);
    angLayout.addDouble("wrist raw abs encoder", wristEncoder::getAbsolutePosition);

    angLayout.addDouble("shoulder abs encoder", this::getShoulderPosition);
    angLayout.addDouble("wrist abs encoder", this::getWristPosition);

    angLayout.addDouble("shoulder abs angle", () -> this.getShoulderPosition() * 360);
    angLayout.addDouble("wrist abs angle", () -> this.getWristPosition() * 360);

    angLayout.addDouble("shoulder angle", () -> ticksToShoulderAngle(shoulder.getSelectedSensorPosition()));
    angLayout.addDouble("wrist angle", () -> ticksToWristAngle(wrist.getSelectedSensorPosition()));
 
    // Angles using remap()   
    angLayout.addDouble("wrist remap", () -> remap(wristEncoder.getAbsolutePosition(), ArmConfig.WRIST_REMAP_LIMIT));
    angLayout.addDouble("shoulder remap", () -> remap(shoulderEncoder.getAbsolutePosition(), ArmConfig.SHOULDER_REMAP_LIMIT));

    angLayout.addBoolean("wrist is connected", () -> wristEncoder.isConnected());
    angLayout.addBoolean("shoulder is connected", () -> shoulderEncoder.isConnected());

    // ShuffleboardLayout dynamicLimits = Shuffleboard.getTab("arm")
    // .getLayout("Dynamic Limits", BuiltInLayouts.kGrid)
    // .withSize(2, 3)
    // .withProperties(Map.of("Label position", "TOP"));
    

    // dynamicLimits.addDouble(getName(), null);

  }

  // Moves encoder discontinuity outisde of range to stop values from jumping
  public double remap(double value, double limit) {
    if (value >= 0 && value < limit) {
      return value + 1;
    } else {
      return value;
    }
  }

  @Override
  public void periodic() {
    if (!RobotController.isSysActive()) {
      holdCurrentPosition();
    }
    // double shoulderForwardLimit = anglesToShoulderSensorPosition(ArmConfig.SHOULDER_FORWARD_LIMIT);
    // double shoulderReverseLimit = anglesToShoulderSensorPosition(ArmConfig.SHOULDER_REVERSE_LIMIT);
    // double wristForwardLimit = anglesToWristSensorPosition(ArmConfig.WRIST_FORWARD_LIMIT);
    // double wristReverseLimit = anglesToWristSensorPosition(ArmConfig.WRIST_REVERSE_LIMIT);
    // if ((wrist.getSelectedSensorPosition() >= wristForwardLimit)
    //     || (wrist.getSelectedSensorPosition() <= wristReverseLimit)) {
    //   controller.runRumble(RumbleVariables.medium);
    // } else if ((shoulder.getSelectedSensorPosition() >= shoulderForwardLimit)
    //     || (shoulder.getSelectedSensorPosition() <= shoulderReverseLimit)) {
    //   controller.runRumble(RumbleVariables.medium);
    // } else {
    //   controller.runRumble(RumbleVariables.off);
    // }

    updatePID();
  }

  public void holdCurrentPosition() {
    double currentWristPosition = wrist.getSelectedSensorPosition();
    double currentShoulderPosition = shoulder.getSelectedSensorPosition();

    wrist.set(TalonFXControlMode.Position, currentWristPosition);
    shoulder.set(TalonFXControlMode.Position, currentShoulderPosition);
  }

  public ArrayList<TalonFX> geTalonFXs() {

    ArrayList<TalonFX> musicList = new ArrayList<>();
    musicList.add(shoulder);
    musicList.add(wrist);

    return musicList;
  }
}
