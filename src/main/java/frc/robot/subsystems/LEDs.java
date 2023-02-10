package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private CANdle candle = new CANdle(Constants.LEDS.CANDLE);
  private double m_lastDisconectTime = 0.0;
  private double m_lastBrownedOutTime = 0.0;

  public enum DesieredState {
    RAINBOW_ANIMATION,
    STROBE_ANIMATION,
    FULL_LEDS,
    OFF,
  }

  private DesieredState desieredState = DesieredState.OFF;

  private enum SystemState {
    RAINBOW_ANIMATION,
    STROBE_ANIMATION,
    FULL_LEDS,
    DRIVER_STATION_DISCONNECTED,
    BROWNOUT,
    OFF,
  }

  private SystemState systemState = SystemState.OFF;

  public void setDesieredState(DesieredState desieredState) {

    this.desieredState = desieredState;
  }

  private void transitionSystemState(SystemState newsystemState) {

    if (this.systemState == newsystemState) {

      return;
    }
    this.systemState = newsystemState;

    switch (newsystemState) {
      case FULL_LEDS: {
        setSolidColor();
      }
        break;

      case DRIVER_STATION_DISCONNECTED: {

        setPurple();

      }

      case RAINBOW_ANIMATION: {
        setLEDRainAnimationFast();
      }
        break;
      case STROBE_ANIMATION: {
        setLEDStrobeAnimation(0, 0, 0, 0, m_lastBrownedOutTime, 0, 0);
      }
        break;

      case BROWNOUT: {
        setBrown();

      }

      case OFF: {
        setOffLEDs();
      }
        break;
    }
  }

  public LEDs() {

    CANdleConfiguration config = new CANdleConfiguration();
    candle.configAllSettings(config);
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
      }
        break;

      case OFF: {
      }
        break;

      case RAINBOW_ANIMATION: {
      }
        break;
      case STROBE_ANIMATION: {
      }
        break;
    }

  }

  // private void setLEDRainAnimation() {

  //   RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.3, 164);
   //   candle.animate(rainbowAnim);
// }

private void setPurple() {

    candle.setLEDs(0, 0, 0, 0, 0, 0);

  }

  private void setLEDRainAnimationFast() {

    RainbowAnimation rAnimation = new RainbowAnimation(1, 1, 164);
    candle.animate(rAnimation);
  }

  private void setSolidColor() {
    candle.setLEDs(255, 0, 225, 0, 0, 164);
  }

  public void setOffLEDs() {
    candle.animate(null);
    candle.setLEDs(0, 0, 0, 0, 0, 164);
  }

  private void setBrown() {

    candle.setLEDs(100, 50, 0, 0, 0, 164);

  }

  private void setLEDStrobeAnimation(
      int r, int g, int b, int w, double speed, int numLed, int ledOffset) {
    StrobeAnimation strobeAnimation = new StrobeAnimation(r, g, b, w, speed, numLed, ledOffset);
    candle.animate(strobeAnimation);
  }
}


