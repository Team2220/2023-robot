package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConfig;
import frc.robot.commands.Arm.SetArmState;
import java.util.Map;

public class Arm extends SubsystemBase {

  private TunableTalonFX angularShoulder;

  private TunableTalonFX angularWrist;

  private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(
    ArmConfig.WRIST_DUTYENCODER
  );
  private DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(
    ArmConfig.SHOULDER_DUTYENCODER
  );

  private TalonFX wrist = new TalonFX(ArmConfig.WRIST_TALONFX);
  private TalonFX shoulder = new TalonFX(ArmConfig.SHOULDER_TALONFX);

  ShuffleboardTab arm = Shuffleboard.getTab("arm");

  /** Config Objects for motor controllers */
  TalonFXConfiguration wristConfig = new TalonFXConfiguration();

  TalonFXConfiguration shoulderConfig = new TalonFXConfiguration();

  public Arm() {
    var wristConfig = new TunableTalonFX.Config(
      ArmConfig.WRIST_DUTYENCODER,
      ArmConfig.WRIST_TALONFX,
      "Wrist",
      ArmConfig.WRIST_GEAR_RATIO,
      ArmConfig.WRIST_INVERTED,
      ArmConfig.WRIST_REMAP_LIMIT,
      ArmConfig.WRIST_ENCODER_OFFSET);
      
      wristConfig.talonRef = ArmConfig.WRIST_REF;
      wristConfig.P = .1;
      wristConfig.D = .2;
      wristConfig.voltageCompSaturation = 10;
      wristConfig.tunableDoubleEnabled = false;
      wristConfig.forwardSoftLimitEnable = true;
      wristConfig.forwardSoftLimitThreshold = ArmConfig.WRIST_REVERSE_LIMIT;
      wristConfig.reverseSoftLimitEnable = true;
      wristConfig.reverseSoftLimitThreshold = ArmConfig.WRIST_FORWARD_LIMIT;
      wristConfig.acceleration = 200;
      wristConfig.cruiseVelocity = 200;
      wristConfig.statorCurrentLimitEnabledDefaultVal = true;
      wristConfig.statorCurrentLimitDefaultVal = 33;
      wristConfig.supplyCurrentLimitEnabledDefaultVal = true;
      wristConfig.supplyCurrentLimitDefaultVal = 20;

      
    var shoulderConfig = new TunableTalonFX.Config(
      ArmConfig.SHOULDER_DUTYENCODER,
      ArmConfig.SHOULDER_TALONFX,
      "Shoulder",
      ArmConfig.SHOULDER_GEAR_RATIO,
      ArmConfig.SHOULDER_INVERTED,
      ArmConfig.SHOULDER_REMAP_LIMIT,
      ArmConfig.SHOULDER_ENCODER_OFFSET);
      
      shoulderConfig.talonRef = ArmConfig.SHOULDER_REF;
      shoulderConfig.P = .1;
      shoulderConfig.D = .2;
      shoulderConfig.voltageCompSaturation = 10;
      shoulderConfig.tunableDoubleEnabled = false;
      shoulderConfig.forwardSoftLimitEnable = true;
      shoulderConfig.forwardSoftLimitThreshold = ArmConfig.SHOULDER_REVERSE_LIMIT;
      shoulderConfig.reverseSoftLimitEnable = true;
      shoulderConfig.reverseSoftLimitThreshold = ArmConfig.SHOULDER_FORWARD_LIMIT;
      shoulderConfig.acceleration = 200;
      shoulderConfig.cruiseVelocity = 200;
      shoulderConfig.statorCurrentLimitEnabledDefaultVal = true;
      shoulderConfig.statorCurrentLimitDefaultVal = 33;
      shoulderConfig.supplyCurrentLimitEnabledDefaultVal = true;
      shoulderConfig.supplyCurrentLimitDefaultVal = 20;

      angularShoulder = new TunableTalonFX(shoulderConfig);
      angularWrist = new TunableTalonFX(wristConfig);

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
