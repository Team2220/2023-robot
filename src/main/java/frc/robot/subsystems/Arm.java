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


  public void setWristPercentOutput(double value) {
    angularWrist.setTalonPercentOutput(value);
  }

  public void setShoulderPercentOutput(double value) {
    angularShoulder.setTalonPercentOutput(value);
  }

  public void setWristAngle(double angle) {
    angularWrist.setTalonAngle(angle);
  }

  public void setShoulderAngle(double angle) {
    angularShoulder.setTalonAngle(angle);
  }

  public double getShoulderPosition() {
    return angularShoulder.getTalonPosition();
  }

  public double getWristPosition() {
    return angularWrist.getTalonPosition();
  }

  public double getMotorShoulderPosition() {
    return angularShoulder.getMotorTalonPosition();
  }

  public double getMotorWristPosition() {
    return angularWrist.getMotorTalonPosition();
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
  }

  // Moves encoder discontinuity outisde of range to stop values from jumping

  public void holdCurrentPosition() {
    angularWrist.holdCurrentPosition();
    angularShoulder.holdCurrentPosition();
  }
}
