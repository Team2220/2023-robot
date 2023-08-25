package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ArmConfig;
import frc.twilight.tunables.TunableDouble;
import frc.twilight.tunables.TunableBoolean;
import java.util.ArrayList;
import java.util.Map;

class AngularTalonFX {

  private DutyCycleEncoder talonEncoder;
  private TalonFX talonFX;

  private double gearRatio;
  private double remapLimit;
  private double encoderOffset;

  private double talonRef;

  private String name;

  public class Config {
    
  }

  public AngularTalonFX(
    int dutyEncoder,
    int talonId,
    String name,
    boolean tunableDoubleEnabled,
    double gearRatio,
    boolean inverted,
    double remapLimit,
    double encoderOffset,
    double talonRef,
    // default values
    boolean brakeMode, // true
    double P, //.1
    double I, //0
    double D, //.2
    int voltageCompSaturation, //10
    boolean forwardSoftLimitEnable, //false
    double forwardSoftLimitThreshold, //10
    double reverseSoftLimitThreshold, //10
    boolean reverseSoftLimitEnable, //false
    int acel, //200
    int cruiseVel, //200
    boolean statorCurrentLimitEnabledDefaultVal, //true
    int statorCurrentLimitDefaultVal, //33
    boolean supplyCurrentLimitEnabledDefaultVal, //true
    int supplyCurrentLimitDefaultVal //20
  ) {
    this.gearRatio = gearRatio;
    this.remapLimit = remapLimit;
    this.encoderOffset = encoderOffset;
    this.name = name;
    this.talonRef = talonRef;

    talonEncoder = new DutyCycleEncoder(dutyEncoder);
    talonFX = new TalonFX(talonId);

    talonFX.configAllSettings(new TalonFXConfiguration());
     
    new TunableBoolean(name + "Brake", brakeMode, tunableDoubleEnabled, name, value -> {
      talonFX.setNeutralMode(value ? NeutralMode.Brake : NeutralMode.Coast);
    });

    new TunableDouble(name + "P", P, tunableDoubleEnabled, name, value -> {
      talonFX.config_kP(0, value);
    });

    new TunableDouble(name + "I", I, tunableDoubleEnabled, name, value -> {
      talonFX.config_kI(0, value);
    });
    
    new TunableDouble(name + "D", D, tunableDoubleEnabled, name, value -> {
      talonFX.config_kD(0, value);
    });
    
    new TunableDouble("VoltageCompSaturation", voltageCompSaturation, tunableDoubleEnabled, name, value -> {
      talonFX.configVoltageCompSaturation(value);
    });

    new TunableBoolean("ForwardSoftLimitEnable", forwardSoftLimitEnable, tunableDoubleEnabled, name, value -> {
      talonFX.configForwardSoftLimitEnable(value);
    });

    new TunableDouble("ForwardSoftLimitThreshold", forwardSoftLimitThreshold, tunableDoubleEnabled, name, value -> {
      talonFX.configForwardSoftLimitThreshold(anglesToTalonSensorPosition(value));
    });
    
    new TunableDouble("ReverseSoftLimitThreshold", reverseSoftLimitThreshold, tunableDoubleEnabled, name, value -> {
      talonFX.configForwardSoftLimitThreshold(anglesToTalonSensorPosition(value));
    });

    new TunableBoolean("ReverseSoftLimitEnable", reverseSoftLimitEnable,  tunableDoubleEnabled, name, value -> {
      talonFX.configReverseSoftLimitEnable(value);
    });

    new TunableDouble(name + "Acel", acel, tunableDoubleEnabled, name, value -> {
      talonFX.configMotionAcceleration(degreesPerSecondToEncoderTicks(value));
    });

    new TunableDouble(name + "CruiseVel", cruiseVel, tunableDoubleEnabled, name, value -> {
      talonFX.configMotionCruiseVelocity(degreesPerSecondToEncoderTicks(value));
    });

    new TunableBoolean(name + "inverted", inverted, tunableDoubleEnabled, name, value -> {
      talonFX.setInverted(value);
    });

    var statorCurrentLimitEnabled = new TunableBoolean(name + "statorCurrentLimitEnabled", statorCurrentLimitEnabledDefaultVal, tunableDoubleEnabled,
        name);
    var statorCurrentLimit = new TunableDouble(name + "statorCurrentLimit", statorCurrentLimitDefaultVal, tunableDoubleEnabled, name);

    statorCurrentLimitEnabled.addChangeListener( value -> {
      setStator(statorCurrentLimit.getValue(), value);
    });

    statorCurrentLimit.addChangeListener( value -> {
      setStator(value, statorCurrentLimitEnabled.getValue());
    });

    var supplyCurrentLimitEnabled = new TunableBoolean( name + "supplyCurrentLimitEnabled", supplyCurrentLimitEnabledDefaultVal, tunableDoubleEnabled, name);
    var supplyCurrentLimit = new TunableDouble(name + "supplyCurrentLimit", supplyCurrentLimitDefaultVal, tunableDoubleEnabled, name);

    supplyCurrentLimitEnabled.addChangeListener(value -> {
      setSupply(statorCurrentLimit.getValue(), value);
    });

    supplyCurrentLimit.addChangeListener(value -> {
      setSupply(value, supplyCurrentLimitEnabled.getValue());
    });

    Shuffleboard.getTab(name).addDouble("Temperature (in Celcius)", talonFX::getTemperature).withWidget(BuiltInWidgets.kGraph);

    EventLoops.oncePerSec.bind(this::checkTemp);
    EventLoops.oncePerMin.bind(this::isStalledLogger);

    setTalonFromAbsEncoder();
    setUpTestCommands();

    CommandScheduler.getInstance().getDefaultButtonLoop().bind(() -> {
      if (!RobotController.isSysActive()) {
        holdCurrentPosition();
      }
    });
  }

  private void setSupply(double supplyCurrentLimit, boolean supplyEnable) {
    SupplyCurrentLimitConfiguration supplyConfig = new SupplyCurrentLimitConfiguration(); 
    supplyConfig.currentLimit = 20;
    supplyConfig.enable = true;
    talonFX.configSupplyCurrentLimit(supplyConfig);
  }

  private void setStator(double currentLimit, boolean enable) {
    StatorCurrentLimitConfiguration statorConfig = new StatorCurrentLimitConfiguration();
    statorConfig.currentLimit = currentLimit;
    statorConfig.enable = enable;
    talonFX.configStatorCurrentLimit(statorConfig);
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
  
  TunableDouble deTime = new TunableDouble("debounceTime", 0.1, true, name);
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
  }

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
