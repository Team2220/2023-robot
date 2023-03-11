package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Leds.SetLedsStates;

public class LEDs extends SubsystemBase {
  private CANdle left = new CANdle(Constants.LEDS.LEFT);
  private CANdle right = new CANdle(Constants.LEDS.RIGHT);
  private double m_lastDisconectTime = 0.0;
  private double m_lastBrownedOutTime = 0.0;
  private double m_startTime = 25.0;
  private double m_startWantingCube = 0;
  private double m_startWantingCone = 0;

  public enum DesieredState {
    RAINBOW_ANIMATION,
    STROBE_ANIMATION,
    FULL_LEDS,
    OFF,
    WANT_CUBE,
    WANT_CONE,
  }

  private DesieredState desieredState = DesieredState.OFF;

  private enum SystemState {
    RAINBOW_ANIMATION,
    STROBE_ANIMATION,
    FULL_LEDS,
    DRIVER_STATION_DISCONNECTED,
    BROWNOUT,
    OFF,
    NOTHING_IN_AUTO,
    TWENTY_SEC_LEFT,
    WANTING_CUBE,
    WANTING_CONE,
  }

  private SystemState systemState = SystemState.OFF;

  public void setDesieredState(DesieredState desieredState) {
    switch (desieredState) {
      case WANT_CONE:
        m_startWantingCone = Timer.getFPGATimestamp();
        break;
      case WANT_CUBE:
        m_startWantingCube = Timer.getFPGATimestamp();
        break;
      default:
        break;
    }
    this.desieredState = desieredState;
  }

  private void transitionSystemState(SystemState newsystemState) {

    if (this.systemState == newsystemState) {

      return;
    }
    DataLogManager.log("Setting LEDs from " + this.systemState + " to " + newsystemState);
    this.systemState = newsystemState;
    setOffLEDs();
    switch (newsystemState) {
      case FULL_LEDS: {
        setSolidColor();
      }
        break;

      case DRIVER_STATION_DISCONNECTED: {
        setBlue();
      }
        break;

      case RAINBOW_ANIMATION: {
        setLEDRainAnimationFast();
      }
        break;
      case STROBE_ANIMATION: {
        setLEDStrobeAnimation(98, 56, 50, 0, .5, 164, 0);
      }
        break;

      case BROWNOUT: {
        setBrown();
      }
        break;
      case OFF: {
        setOffLEDs();
      }
        break;

      case NOTHING_IN_AUTO: {
      }
        break;

      case TWENTY_SEC_LEFT: {

        setLeds20SecsLeft();
      }
        break;
      case WANTING_CONE: {
        setLEDStrobeAnimation(249, 149, 2, 0, .5, 164, 0);
        break;
      }
      case WANTING_CUBE: {
        setLEDStrobeAnimation(66, 5, 188, 0, .5, 164, 0);
        break;
      }
    }
  }

  public LEDs() {

    CANdleConfiguration config = new CANdleConfiguration();
    left.configAllSettings(config);
    right.configAllSettings(config);
    setUpTestCommands();
  }

  private void switchDesieredState() {

    switch (desieredState) {
      case RAINBOW_ANIMATION: {
        transitionSystemState(SystemState.RAINBOW_ANIMATION);
      }
        break;
      case FULL_LEDS: {
        transitionSystemState(SystemState.FULL_LEDS);
      }
        break;

      case OFF: {
        transitionSystemState(SystemState.OFF);
      }
        break;

      case STROBE_ANIMATION: {
        transitionSystemState(SystemState.STROBE_ANIMATION);
      }
        break;
      case WANT_CONE: {
        transitionSystemState(SystemState.WANTING_CONE);
        break;
      }
      case WANT_CUBE: {
        transitionSystemState(SystemState.WANTING_CUBE);
        break;

      }
    }
  }

  @Override
  public void periodic() {

    if (RobotController.isBrownedOut()) {

      m_lastBrownedOutTime = Timer.getFPGATimestamp();
      transitionSystemState(SystemState.BROWNOUT);
    }

    if (!DriverStation.isDSAttached()) {

      m_lastDisconectTime = Timer.getFPGATimestamp();
      transitionSystemState(SystemState.DRIVER_STATION_DISCONNECTED);
    }

    switch (systemState) {
      case DRIVER_STATION_DISCONNECTED: {
        if (Timer.getFPGATimestamp() > m_lastDisconectTime + 5) {
          switchDesieredState();
        }
      }
        break;

      case BROWNOUT: {
        if (Timer.getFPGATimestamp() > m_lastBrownedOutTime + 5) {
          switchDesieredState();
        }
      }
        break;

      case FULL_LEDS: {
        switchDesieredState();
      }
        break;

      case OFF: {
        switchDesieredState();
      }
        break;

      case RAINBOW_ANIMATION: {
        switchDesieredState();
      }
        break;
      case STROBE_ANIMATION: {
        switchDesieredState();
      }
        break;
      case NOTHING_IN_AUTO: {

      }
        break;

      case TWENTY_SEC_LEFT: {
        if (Timer.getFPGATimestamp() > m_startTime - 5) {
          switchDesieredState();
        }
      }
        break;
      case WANTING_CONE: {
        if (Timer.getFPGATimestamp() > m_startWantingCone + 5) {
          transitionSystemState(SystemState.OFF);
        }
      }
        break;
      case WANTING_CUBE: {
        if (Timer.getFPGATimestamp() > m_startWantingCube + 5) {
          transitionSystemState(SystemState.OFF);
        }
      }
        break;
    }
  }

  // private void setLEDRainAnimation() {

  // RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.3, 164);
  // candle.animate(rainbowAnim);
  // }

  private void setBlue() {

    SingleFadeAnimation singleFadeAnimation = new SingleFadeAnimation(0, 0, 100, 0, .5, 164);
    left.animate(singleFadeAnimation);
    right.animate(singleFadeAnimation);
  }

  private void setLEDRainAnimationFast() {

    RainbowAnimation rAnimation = new RainbowAnimation(1, 1, 164);
    left.animate(rAnimation);
    right.animate(rAnimation);
  }

  private void setSolidColor() {
    left.setLEDs(255, 0, 225, 0, 0, 164);
    right.setLEDs(255, 0, 225, 0, 0, 164);
  }

  public void setOffLEDs() {
    left.animate(null);
    left.setLEDs(0, 0, 0, 0, 0, 164);
    right.animate(null);
    right.setLEDs(0, 0, 0, 0, 0, 164);
  }

  private void setBrown() {

    StrobeAnimation strobeAnimation = new StrobeAnimation(64, 36, 0, 0, 0.1, 164);
    left.animate(strobeAnimation);
    right.animate(strobeAnimation);
  }

  private void setLEDStrobeAnimation(
      int r, int g, int b, int w, double speed, int numLed, int ledOffset) {
    StrobeAnimation strobeAnimation = new StrobeAnimation(r, g, b, w, speed, numLed, ledOffset);
    left.animate(strobeAnimation);
    right.animate(strobeAnimation);
  }

  private void setLeds20SecsLeft() {
    StrobeAnimation timedStrobeAnimation = new StrobeAnimation(0, 0, 0, 0, m_lastBrownedOutTime, 0, 0);
    left.animate(timedStrobeAnimation);
    right.animate(timedStrobeAnimation);
  }

  public void setUpTestCommands() {
    // Arm States
    ShuffleboardLayout stateLayout = Shuffleboard.getTab("leds")
        .getLayout("States", BuiltInLayouts.kList)
        .withSize(2, 3)
        .withProperties(Map.of("Label position", "HIDDEN"));

    for (DesieredState state : DesieredState.values()) {
      stateLayout.add(state.name(), new SetLedsStates(state, this));
    }

    // Everything else
    ShuffleboardLayout angLayout = Shuffleboard.getTab("leds")
        .getLayout("Angles", BuiltInLayouts.kGrid)
        .withSize(2, 3)
        .withProperties(Map.of("Label position", "TOP"));

    angLayout.addString("DesieredStatew", () -> this.desieredState.name());
    angLayout.addString("SystemState", () -> this.systemState.name());
  }
}
