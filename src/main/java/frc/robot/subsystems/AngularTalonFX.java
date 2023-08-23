package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConfig;
import frc.robot.commands.Arm.SetArmState;
import frc.twilight.tunables.TunableDouble;
import frc.twilight.tunables.TunableBoolean;
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
  private final TunableBoolean inverted;
  
  private final TunableDouble voltageCompSaturation;
  private final TunableBoolean configForwardSoftLimitEnable;
  private final TunableBoolean configReverseSoftLimitEnable;
  private final TunableDouble configForwardSoftLimitThreshold;
  private final TunableDouble configReverseSoftLimitThreshold;

  private final TunableBoolean brake;

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

    brake = new TunableBoolean(name + "Brake", true, tunableDoubleEnabled, name);

    talonP = new TunableDouble(name + "P", 0.1, tunableDoubleEnabled, name);
    talonI = new TunableDouble(name + "I", 0, tunableDoubleEnabled, name);
    talonD = new TunableDouble(name + "D", 0.2, tunableDoubleEnabled, name);
    
    voltageCompSaturation = new TunableDouble("VoltageCompSaturation", 10, tunableDoubleEnabled, name);
    configForwardSoftLimitEnable = new TunableBoolean("configForwardSoftLimitEnable", false, tunableDoubleEnabled,
        name);
    configForwardSoftLimitThreshold = new TunableDouble("configForwardSoftLimitThreshold", 10, tunableDoubleEnabled, name);
    configReverseSoftLimitThreshold = new TunableDouble("configReverseSoftLimitThreshold", 10, tunableDoubleEnabled,
        name);
    configReverseSoftLimitEnable = new TunableBoolean("configReverseSoftLimitEnable", false,  tunableDoubleEnabled,
        name);


    acel = new TunableDouble(name + "Acel", 200, tunableDoubleEnabled, name);
    cruiseVel =
      new TunableDouble(name + "CruiseVel", 200, tunableDoubleEnabled, name);

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

    voltageCompSaturation.addChangeListener(value -> {
      talonFX.configVoltageCompSaturation(value);
    });
    
    configForwardSoftLimitThreshold.addChangeListener(value -> {
      anglesToTalonSensorPosition(value);
    });

    configReverseSoftLimitThreshold.addChangeListener(value -> {
      anglesToTalonSensorPosition(value);
    });

    configForwardSoftLimitEnable.addChangeListener(value -> {
      talonFX.configForwardSoftLimitEnable(value);
    });

    configReverseSoftLimitEnable.addChangeListener(value -> {
      talonFX.configReverseSoftLimitEnable(value);
    });

    talonFX.configAllSettings(talonConfig);

    SupplyCurrentLimitConfiguration supplyConfig = new SupplyCurrentLimitConfiguration();
    supplyConfig.currentLimit = 20;
    supplyConfig.enable = true;
    talonFX.configSupplyCurrentLimit(supplyConfig);

    this.inverted = new TunableBoolean(name + "inverted", inverted, tunableDoubleEnabled, name);

    Shuffleboard.getTab(name).addDouble("Temperature", talonFX::getTemperature).withWidget(BuiltInWidgets.kGraph);

    EventLoops.oncePerSec.bind(this::checkTemp);
    EventLoops.oncePerMin.bind(this::isStalledLogger);

    StatorCurrentLimitConfiguration statorConfig = new StatorCurrentLimitConfiguration();
    statorConfig.currentLimit = 33;
    statorConfig.enable = true;
    talonFX.configStatorCurrentLimit(statorConfig);

    talonFX.setNeutralMode(NeutralMode.Brake);




    brake.addChangeListener(value -> {
      if (value == true) {
        talonFX.setNeutralMode(NeutralMode.Brake);
      } else {
        talonFX.setNeutralMode(NeutralMode.Coast);
      }
    });

    
    this.inverted.addChangeListener(value -> {
      talonFX.setInverted(value);
    });

    setTalonFromAbsEncoder();

    setUpTestCommands();
  }

  private double degreesPerSecondToEncoderTicks(double angle) {
    double gfx = ((angle / 360.0) * gearRatio) *
        ArmConfig.TALONFX_ENCODER_TICKS *
        1.0 /
        10.0;
    return gfx;
  }

  public void setTalonFromAbsEncoder() {
    double talonOffset =
      getTalonPosition() * (gearRatio) * (ArmConfig.TALONFX_ENCODER_TICKS);
    talonFX.setSelectedSensorPosition(talonOffset);
    System.out.println("!!! name = " + name + "\n" + "!!! Talon offset = " + talonOffset + "\n !!! get_talon_position = " + getTalonPosition() + "\n !!! talon_offset = " + ticksToTalonAngle(talonOffset));
  }

  public double getTalonPosition() {
    return (remap(talonEncoder.getAbsolutePosition(), remapLimit) - encoderOffset);
  }

  public double CtoF(double temp) {
    return (temp * 9 / 5) + 32;
  }

  public double CtoK(double temp) {
    return temp + 273.15;
  }

  public void checkTemp() {
    double tempCel = talonFX.getTemperature();

    if (tempCel > 90) {
      DataLogManager.log("Temperature is concerning. Temperature is: " + tempCel + " °C or " + CtoF(tempCel) + " °F or "
          + CtoK(tempCel) + " K.");
    }
  }

  private boolean isStalledInternal() {
    if (talonFX.getStatorCurrent() >= 75) {
      double velocity = talonFX.getSelectedSensorVelocity();
      return velocity <= 5;
    } else {
      return false;
    }
  }
  
  TunableDouble deTime = new TunableDouble("debounceTime", 0.1, true);
  Debouncer debouncer = new Debouncer(deTime.getValue(), Debouncer.DebounceType.kBoth);

  public void isStalledLogger() {
    if (debouncer.calculate(isStalledInternal()) == true) {
      DataLogManager.log("Motor is stalled.");
    }
  }

  public boolean isStalled() {
    return debouncer.calculate(isStalledInternal());
  }

  public double remap(double value, double limit) {
    if (value >= 0 && value < limit) {
      return value + 1;
    } else {
      return value;
    }
  }

  public void overrideTalonSoftLimits(boolean enabled) {
    DataLogManager.log("override" + name + "SoftLimits " + enabled);
    talonFX.overrideSoftLimitsEnable(enabled);
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



  public void setTalonPosition(double talonAng) {
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

  public double getMotorTalonPosition() {
    return ticksToTalonAngle(talonFX.getSelectedSensorPosition());
  }

  public void setUpTestCommands() {
    // Test Positions
    ShuffleboardLayout testPositionLayout = Shuffleboard
      .getTab(name)
      .getLayout("Test Positions", BuiltInLayouts.kList)
      .withSize(2, 4)
      .withProperties(Map.of("Label position", "HIDDEN"));

    testPositionLayout.add(
      "Zero" + name,
      new InstantCommand(() -> talonFX.setSelectedSensorPosition(0))
        .withName("Zero" + name)
    );

    testPositionLayout.add(
      "set" + name + "FromEncoder",
      new InstantCommand(() -> setTalonFromAbsEncoder())
        .withName("set" + name + "FromEncoder")
    );

    testPositionLayout.add(
      "Reference" + name,
      new InstantCommand(() -> setTalonToReferenceAngle())
        .withName("Reference" + name)
    );

    testPositionLayout.add(
      name + "=90",
      new InstantCommand(() -> setTalonAngle(90)).withName(name + "=90")
    );

    testPositionLayout.add(
      name + "=0",
      new InstantCommand(() -> setTalonAngle(0)).withName(name + "=0")
    );
    testPositionLayout.add(
      name + "=-90",
      new InstantCommand(() -> setTalonAngle(-90)).withName(name + "=-90")
    );

    // Everything else
    ShuffleboardLayout angLayout = Shuffleboard
      .getTab(name)
      .getLayout("Angles", BuiltInLayouts.kGrid)
      .withSize(3, 4)
      .withProperties(Map.of("Label position", "TOP"));

    angLayout.addDouble(
      name + " raw abs encoder",
      talonEncoder::getAbsolutePosition
    );

    angLayout.addDouble(name + " abs encoder", this::getTalonPosition);

    angLayout.addDouble(
      name + " abs angle",
      () -> this.getTalonPosition() * 360
    );

    angLayout.addDouble(
      name + " angle",
      () -> ticksToTalonAngle(talonFX.getSelectedSensorPosition())
    );
    // Angles using remap()
    angLayout.addDouble(
      name + " remap",
      () -> remap(talonEncoder.getAbsolutePosition(), remapLimit)
    );

    angLayout.addBoolean(
      name + " is connected",
      () -> talonEncoder.isConnected()
    );
    // ShuffleboardLayout dynamicLimits = Shuffleboard.getTab("arm")
    // .getLayout("Dynamic Limits", BuiltInLayouts.kGrid)
    // .withSize(2, 3)
    // .withProperties(Map.of("Label position", "TOP"));

    // dynamicLimits.addDouble(getName(), null);

  }

  // if (!RobotController.isSysActive()) {
  // holdCurrentPosition();
  // }
  // double shoulderForwardLimit =
  // anglesToShoulderSensorPosition(ArmConfig.SHOULDER_FORWARD_LIMIT);
  // double shoulderReverseLimit =
  // anglesToShoulderSensorPosition(ArmConfig.SHOULDER_REVERSE_LIMIT);
  // double wristForwardLimit =
  // anglesToWristSensorPosition(ArmConfig.WRIST_FORWARD_LIMIT);
  // double wristReverseLimit =
  // anglesToWristSensorPosition(ArmConfig.WRIST_REVERSE_LIMIT);
  // if ((wrist.getSelectedSensorPosition() >= wristForwardLimit)
  // || (wrist.getSelectedSensorPosition() <= wristReverseLimit)) {
  // controller.runRumble(RumbleVariables.medium);
  // } else if ((shoulder.getSelectedSensorPosition() >= shoulderForwardLimit)
  // || (shoulder.getSelectedSensorPosition() <= shoulderReverseLimit)) {
  // controller.runRumble(RumbleVariables.medium);
  // } else {
  // controller.runRumble(RumbleVariables.off);
  // }

  public void holdCurrentPosition() {
    double currentTalonPosition = talonFX.getSelectedSensorPosition();

    talonFX.set(TalonFXControlMode.Position, currentTalonPosition);
  }

  public ArrayList<TalonFX> geTalonFXs() {
    ArrayList<TalonFX> musicList = new ArrayList<>();
    musicList.add(talonFX);

    return musicList;
  }
}
