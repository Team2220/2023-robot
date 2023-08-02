package frc.robot.subsystems;

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
import java.util.ArrayList;
import java.util.Map;

public class Arm extends SubsystemBase {

  private final boolean tunableDoubleEnabled = true;

  private AngularTalonFX angularShoulder = new AngularTalonFX(
    ArmConfig.SHOULDER_DUTYENCODER,
    ArmConfig.SHOULDER_TALONFX,
    "Shoulder",
    tunableDoubleEnabled,
    ArmConfig.SHOULDER_GEAR_RATIO,
    ArmConfig.SHOULDER_INVERTED,
    ArmConfig.SHOULDER_REMAP_LIMIT,
    ArmConfig.SHOULDER_ENCODER_OFFSET,
    ArmConfig.SHOULDER_FORWARD_LIMIT,
    ArmConfig.SHOULDER_REVERSE_LIMIT,
    ArmConfig.SHOULDER_REF
  );

  private AngularTalonFX angularWrist = new AngularTalonFX(
    ArmConfig.WRIST_DUTYENCODER,
    ArmConfig.WRIST_TALONFX,
    "Wrist",
    tunableDoubleEnabled,
    ArmConfig.WRIST_GEAR_RATIO,
    ArmConfig.WRIST_INVERTED,
    ArmConfig.WRIST_REMAP_LIMIT,
    ArmConfig.WRIST_ENCODER_OFFSET,
    ArmConfig.WRIST_FORWARD_LIMIT,
    ArmConfig.WRIST_REVERSE_LIMIT,
    ArmConfig.WRIST_REF
  );

  private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(
    ArmConfig.WRIST_DUTYENCODER
  );
  private DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(
    ArmConfig.SHOULDER_DUTYENCODER
  );

  private TalonFX wrist = new TalonFX(ArmConfig.WRIST_TALONFX);
  private TalonFX shoulder = new TalonFX(ArmConfig.SHOULDER_TALONFX);

  ShuffleboardTab arm = Shuffleboard.getTab("arm");

  private final TunableDouble wristP = new TunableDouble(
    "wristP",
    0.1,
    tunableDoubleEnabled
  );
  private final TunableDouble wristI = new TunableDouble(
    "wristI",
    0,
    tunableDoubleEnabled
  );
  private final TunableDouble wristD = new TunableDouble(
    "wristD",
    0.2,
    tunableDoubleEnabled
  );

  private double oldWristP = wristP.getValue();
  private double oldWristI = wristI.getValue();
  private double oldWristD = wristD.getValue();

  private final TunableDouble shoulderP = new TunableDouble(
    "shoulderP",
    0.1,
    tunableDoubleEnabled
  );
  private final TunableDouble shoulderI = new TunableDouble(
    "shoulderI",
    0,
    tunableDoubleEnabled
  );
  private final TunableDouble shoulderD = new TunableDouble(
    "shoulderD",
    0.2,
    tunableDoubleEnabled
  );

  private final TunableDouble shoulderCruiseVel = new TunableDouble(
    "shoulderCruiseVel",
    100,
    tunableDoubleEnabled
  );
  private final TunableDouble shoulderAcel = new TunableDouble(
    "shoulderAcel",
    200,
    tunableDoubleEnabled
  );

  private final TunableDouble wristCruiseVel = new TunableDouble(
    "wristCruiseVel",
    200,
    tunableDoubleEnabled
  );
  private final TunableDouble wristAcel = new TunableDouble(
    "wristAcel",
    200,
    tunableDoubleEnabled
  );

  private double oldShoulderP = shoulderP.getValue();
  private double oldShoulderI = shoulderI.getValue();
  private double oldShoulderD = shoulderD.getValue();

  /** Config Objects for motor controllers */
  TalonFXConfiguration wristConfig = new TalonFXConfiguration();

  TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();

  public Arm() {
    setUpTestCommands();
  }

  public void setShoulderFromAbsEncoder() {
    angularShoulder.setTalonFromAbsEncoder();
  }

  public void setWristFromAbsEncoder() {
    angularWrist.setTalonFromAbsEncoder();
  }

  public void overrideWristSoftLimits(boolean enabled) {
    angularWrist.overrideTalonSoftLimits(enabled);
  }

  public void overrideShoulderSoftLimits(boolean enabled) {
    angularShoulder.overrideTalonSoftLimits(enabled);
  }

  public double anglesToWristSensorPosition(double angle) {
    double posValue = angularWrist.anglesToTalonSensorPosition(angle);
    return posValue;
  }

  public double anglesToShoulderSensorPosition(double angle) {
    double posValue = angularShoulder.anglesToTalonSensorPosition(angle);
    return posValue;
  }

  public double ticksToWristAngle(double ticks) {
    return angularWrist.ticksToTalonAngle(ticks);
  }

  public double ticksToShoulderAngle(double ticks) {
    return angularShoulder.ticksToTalonAngle(ticks);
  }

  /** Arm enum for arm stataes */
  public enum ArmStates {
    INTAKE(152, 41),
    CALIBRATE(160, 80),
    INTERMEDIATE(60, 80),
    MID_CUBE_NODE(64, -82),
    HIGH_CUBE_NODE(63, -32),
    HIGH_CUBE_AUTO(105, 70),
    MID_CONE_NODE(58, -78),
    HIGH_CONE_NODE(57.5, -27),
    TRANSIT(165, 135),
    // Starting: Shoulder = 171 , Wrist = 150
    SINGLE_LOADING_STATION(167, 141),
    DOUBLE_LOADING_STATION(54, -63);

    public final double shoulderAngle;
    public final double wristAngle;

    private ArmStates(double shoulderAngle, double wristAngle) {
      this.shoulderAngle = shoulderAngle;
      this.wristAngle = wristAngle;
    }
  }

  /** arm states */
  public void setArmState(ArmStates newState) {
    DataLogManager.log("Setting arm state to " + newState.name());
    setPosition(newState.shoulderAngle, newState.wristAngle);
  }

  public boolean atArmState(ArmStates armState) {
    return (
      Math.abs(armState.shoulderAngle - getMotorShoulderPosition()) < 3 &&
      Math.abs(armState.wristAngle - getMotorWristPosition()) < 3
    );
  }

  private void setPosition(double shoulderAng, double wristAng) {
    angularShoulder.setTalonPosition(shoulderAng);
    angularWrist.setTalonPosition(wristAng);
  }

  public void setShoulderToReferenceAngle() {
    shoulder.setSelectedSensorPosition(
      anglesToShoulderSensorPosition(Constants.ArmConfig.SHOULDER_REF)
    );
  }

  public void setWristToReferenceAngle() {
    wrist.setSelectedSensorPosition(
      anglesToWristSensorPosition(Constants.ArmConfig.WRIST_REF)
    );
  }

  public void setWristPercentOutput(double value) {
    wrist.set(TalonFXControlMode.PercentOutput, value);
  }

  public void setShoulderPercentOutput(double value) {
    shoulder.set(TalonFXControlMode.PercentOutput, value);
  }

  public void setWristAngle(double angle) {
    double posValue = anglesToWristSensorPosition(-angle);
    wrist.set(TalonFXControlMode.MotionMagic, posValue);
  }

  public void setShoulderAngle(double angle) {
    double posValue = anglesToShoulderSensorPosition(angle);
    shoulder.set(TalonFXControlMode.MotionMagic, posValue);
  }

  public double getShoulderPosition() {
    return (
      remap(
        shoulderEncoder.getAbsolutePosition(),
        ArmConfig.SHOULDER_REMAP_LIMIT
      ) -
      ArmConfig.SHOULDER_ENCODER_OFFSET
    );
  }

  public double getWristPosition() {
    return (
      remap(wristEncoder.getAbsolutePosition(), ArmConfig.WRIST_REMAP_LIMIT) -
      ArmConfig.WRIST_ENCODER_OFFSET
    );
  }

  public double getMotorShoulderPosition() {
    return ticksToShoulderAngle(shoulder.getSelectedSensorPosition());
  }

  public double getMotorWristPosition() {
    return -ticksToWristAngle(wrist.getSelectedSensorPosition());
  }

  public void setUpTestCommands() {
    // Arm States
    ShuffleboardLayout stateLayout = Shuffleboard
      .getTab("arm")
      .getLayout("States", BuiltInLayouts.kList)
      .withSize(2, 4)
      .withProperties(Map.of("Label position", "HIDDEN"));

    for (ArmStates state : ArmStates.values()) {
      stateLayout.add(state.name(), new SetArmState(state, this));
    }

    // Test Positions
    ShuffleboardLayout testPositionLayout = Shuffleboard
      .getTab("arm")
      .getLayout("Test Positions", BuiltInLayouts.kList)
      .withSize(2, 4)
      .withProperties(Map.of("Label position", "HIDDEN"));

    testPositionLayout.add(
      "ZeroShoulder",
      new InstantCommand(() -> shoulder.setSelectedSensorPosition(0))
        .withName("ZeroShoulder")
    );
    testPositionLayout.add(
      "ZeroWrist",
      new InstantCommand(() -> wrist.setSelectedSensorPosition(0))
        .withName("ZeroWrist")
    );

    testPositionLayout.add(
      "setShoulderFromEncoder",
      new InstantCommand(() -> setShoulderFromAbsEncoder())
        .withName("setShoulderFromEncoder")
    );
    testPositionLayout.add(
      "setWristFromEncoder",
      new InstantCommand(() -> setWristFromAbsEncoder())
        .withName("setWristFromEncoder")
    );

    testPositionLayout.add(
      "ReferenceShoulder",
      new InstantCommand(() -> setShoulderToReferenceAngle())
        .withName("ReferenceShoulder")
    );
    testPositionLayout.add(
      "RefernceWrist",
      new InstantCommand(() -> setWristToReferenceAngle())
        .withName("RefernceWrist")
    );
    testPositionLayout.add(
      "Wrist=90",
      new InstantCommand(() -> setWristAngle(90)).withName("Wrist=90")
    );
    testPositionLayout.add(
      "Wrist=0",
      new InstantCommand(() -> setWristAngle(0)).withName("Wrist=0")
    );
    testPositionLayout.add(
      "Wrist=-90",
      new InstantCommand(() -> setWristAngle(-90)).withName("Wrist=-90")
    );
    testPositionLayout.add(
      "Shoulder=0",
      new InstantCommand(() -> setShoulderAngle(0)).withName("Shoulder=0")
    );
    testPositionLayout.add(
      "Shoulder=90",
      new InstantCommand(() -> setShoulderAngle(90)).withName("Shoulder=90")
    );
    testPositionLayout.add(
      "Shoulder=-90",
      new InstantCommand(() -> setShoulderAngle(-90)).withName("Shoulder=-90")
    );

    // Everything else
    ShuffleboardLayout angLayout = Shuffleboard
      .getTab("arm")
      .getLayout("Angles", BuiltInLayouts.kGrid)
      .withSize(3, 4)
      .withProperties(Map.of("Label position", "TOP"));

    angLayout.addDouble(
      "shoulder raw abs encoder",
      shoulderEncoder::getAbsolutePosition
    );
    angLayout.addDouble(
      "wrist raw abs encoder",
      wristEncoder::getAbsolutePosition
    );

    angLayout.addDouble("shoulder abs encoder", this::getShoulderPosition);
    angLayout.addDouble("wrist abs encoder", this::getWristPosition);

    angLayout.addDouble(
      "shoulder abs angle",
      () -> this.getShoulderPosition() * 360
    );
    angLayout.addDouble("wrist abs angle", () -> this.getWristPosition() * 360);

    angLayout.addDouble(
      "shoulder angle",
      () -> ticksToShoulderAngle(shoulder.getSelectedSensorPosition())
    );
    angLayout.addDouble(
      "wrist angle",
      () -> ticksToWristAngle(wrist.getSelectedSensorPosition())
    );

    // Angles using remap()
    angLayout.addDouble(
      "wrist remap",
      () ->
        remap(wristEncoder.getAbsolutePosition(), ArmConfig.WRIST_REMAP_LIMIT)
    );
    angLayout.addDouble(
      "shoulder remap",
      () ->
        remap(
          shoulderEncoder.getAbsolutePosition(),
          ArmConfig.SHOULDER_REMAP_LIMIT
        )
    );

    angLayout.addBoolean(
      "wrist is connected",
      () -> wristEncoder.isConnected()
    );
    angLayout.addBoolean(
      "shoulder is connected",
      () -> shoulderEncoder.isConnected()
    );
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

  public void holdCurrentPosition() {}
}
