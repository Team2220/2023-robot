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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Arm.SetArmState;
import frc.twilight.LogPowerFaults;
import frc.twilight.tunables.TunableDouble;

public class AngularTalonFX extends SubsystemBase {

  ShuffleboardTab arm = Shuffleboard.getTab("arm");

  private final boolean tunableDoubleEnabled = true;

  private final TunableDouble P;
  private final TunableDouble I;
  private final TunableDouble D;

  private double oldP;
  private double oldI;
  private double oldD;

  private final String name;
  private final int encoderId;
  private final double gearRatio;
  private final int encoderTicks;
  private final double forwardLimit;
  private final double reverseLimit;
  private final int talonRef;
  private final double remapLimit;
  private final double encoderOffset;
  private final int talonFX;
  private final boolean inverted;

  private DutyCycleEncoder talonEncoder;

  private TalonFX talon;


   private double lastTalonAngle = 0;

  TalonFXConfiguration config = new TalonFXConfiguration();

  public AngularTalonFX(
    String name, int encoderId, double gearRatio, int encoderTicks, double forwardLimit, double reverseLimit, 
    int talonRef, double remapLimit, double encoderOffset, int talonFX, boolean inverted
    ) {

        this.name = name;
        this.encoderId = encoderId;
        this.gearRatio = gearRatio;
        this.encoderTicks = encoderTicks;
        this.forwardLimit = forwardLimit;
        this.reverseLimit = reverseLimit;
        this.talonRef = talonRef;
        this.remapLimit = remapLimit;
        this.encoderOffset = encoderOffset;
        this.talonFX = talonFX;
        this.inverted = inverted;

 talonEncoder = new DutyCycleEncoder(encoderId);

 talon = new TalonFX(talonFX);

  P = new TunableDouble(name + "P", 0.1, tunableDoubleEnabled);
  I = new TunableDouble(name + "I", 0, tunableDoubleEnabled);
  D = new TunableDouble(name + "D", 0.2, tunableDoubleEnabled);

  oldP = P.getValue();
  oldI = I.getValue();
  oldD = D.getValue();

     config.motionAcceleration = degreesPerSecondToEncoderTicks(200, gearRatio);
    config.motionCruiseVelocity = degreesPerSecondToEncoderTicks(200, gearRatio);

     talon.configAllSettings(config);

     talon.configVoltageCompSaturation(10);

      talon.setInverted(inverted);

    SupplyCurrentLimitConfiguration supplyConfig = new SupplyCurrentLimitConfiguration();
    supplyConfig.currentLimit = 20;
    supplyConfig.enable = true;
     talon.configSupplyCurrentLimit(supplyConfig);

     StatorCurrentLimitConfiguration config = new StatorCurrentLimitConfiguration();
    config.currentLimit = 33;
    config.enable = true;
    talon.configStatorCurrentLimit(config);

    talon.setNeutralMode(NeutralMode.Brake);

      talon.config_kP(0, P.getValue());
    talon.config_kI(0, I.getValue());
    talon.config_kD(0, D.getValue());

     setTalonFromAbsEncoder();

      talon.configForwardSoftLimitEnable(true);
    talon.configForwardSoftLimitThreshold(
        anglesToTalonSensorPosition(forwardLimit));
    talon.configReverseSoftLimitEnable(true);
    talon.configReverseSoftLimitThreshold(
        anglesToTalonSensorPosition(reverseLimit));

         LogPowerFaults.add(talon);

  }

  public void updatePID() {

    if (P.getValue() != oldP) {
      talon.config_kP(0, P.getValue());
      oldP = P.getValue();
    }

    if (I.getValue() != oldI) {
      talon.config_kI(0, I.getValue());
      oldI = I.getValue();
    }

    if (D.getValue() != oldD) {
      talon.config_kD(0, D.getValue());
      oldD = D.getValue();
    }
  }

        public void setTalonFromAbsEncoder() {
    double offset = getTalonPosition() * (gearRatio) * (encoderTicks);
    talon.setSelectedSensorPosition(offset);
  }

  public void overrideTalonSoftLimits(boolean enabled) {
    DataLogManager.log("override" + name + "SoftLimits " + enabled);
    talon.overrideSoftLimitsEnable(enabled);
  }

  public double anglesToTalonSensorPosition(double angle) {
    double posValue = ((angle / 360.0) * gearRatio) * encoderTicks;

    return posValue;
  }

   public double ticksToTalonAngle(double ticks) {
    double value = ticks / encoderTicks / gearRatio;
    value *= 360;
    return value;
  }

   private double degreesPerSecondToEncoderTicks(double angle, double gearRatio) {
    double gfx = ((angle / 360.0) * gearRatio) * encoderTicks * 1.0 / 10.0;
    return gfx;
  }

  public void setTalonToReferenceAngle() {
    talon.setSelectedSensorPosition(anglesToTalonSensorPosition(talonRef));
  }

  public void setTalonPercentOutput(double value) {
    talon.set(TalonFXControlMode.PercentOutput, value);
  }

  public void setTalonAngle(double angle) {
    lastTalonAngle = angle;
    double posValue = anglesToTalonSensorPosition(-angle);
    talon.set(TalonFXControlMode.MotionMagic, posValue);
  }

   public void changeTalonAngle(double amount) {
    setTalonAngle(lastTalonAngle + amount);
  }

   public double getTalonPosition() {

    return remap(talonEncoder.getAbsolutePosition(), remapLimit) - encoderOffset;
  }

  public double getMotorTalonPosition() {
    return -ticksToTalonAngle(talon.getSelectedSensorPosition());
  }

  public void setUpTestCommands() {

  ShuffleboardLayout angLayout = Shuffleboard.getTab("arm")
        .getLayout("Angles", BuiltInLayouts.kGrid)
        .withSize(3, 4)
        .withProperties(Map.of("Label position", "TOP"));

    angLayout.addDouble(name + " raw abs encoder", talonEncoder::getAbsolutePosition);

    angLayout.addDouble(name + " abs encoder", this::getTalonPosition);

    angLayout.addDouble(name + " abs angle", () -> this.getTalonPosition() * 360);

    angLayout.addDouble(name + " angle", () -> ticksToTalonAngle(talon.getSelectedSensorPosition()));

    // Angles using remap()
    angLayout.addDouble(name + " remap", () -> remap(talonEncoder.getAbsolutePosition(), remapLimit));

    angLayout.addBoolean(name + " is connected", () -> talonEncoder.isConnected());

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
    // if (!RobotController.isSysActive()) {
    // holdCurrentPosition();
    // }
    // double shoulderForwardLimit =
    // anglesToShoulderSensorPosition(forwardLimit);
    // double shoulderReverseLimit =
    // anglesToShoulderSensorPosition(reverseLimit);
    // double wristForwardLimit =
    // anglesToWristSensorPosition(forwardLimit);
    // double wristReverseLimit =
    // anglesToWristSensorPosition(reverseLimit);
    // if ((wrist.getSelectedSensorPosition() >= wristForwardLimit)
    // || (wrist.getSelectedSensorPosition() <= wristReverseLimit)) {
    // controller.runRumble(RumbleVariables.medium);
    // } else if ((shoulder.getSelectedSensorPosition() >= shoulderForwardLimit)
    // || (shoulder.getSelectedSensorPosition() <= shoulderReverseLimit)) {
    // controller.runRumble(RumbleVariables.medium);
    // } else {
    // controller.runRumble(RumbleVariables.off);
    // }

    updatePID();
  }

  public void holdCurrentPosition() {
    double currentTalonPosition = talon.getSelectedSensorPosition();

    talon.set(TalonFXControlMode.Position, currentTalonPosition);
  }

  public ArrayList<TalonFX> geTalonFXs() {

    ArrayList<TalonFX> musicList = new ArrayList<>();
    musicList.add(talon);

    return musicList;
  }
  }
