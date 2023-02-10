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
  CANdle candle = new CANdle(Constants.LEDS.CANDLE);
  public double m_lastDisconectTime = 0.0;
  public double m_lastBrownedOutTime = 0.0;

  enum DesieredState {
    RAINBOW_ANIMATION,
    STROBE_ANIMATION,
    FULL_LEDS,
    OFF,
  }

  DesieredState desieredState = DesieredState.OFF;

  enum SystemState {
    RAINBOW_ANIMATION,
    STROBE_ANIMATION,
    FULL_LEDS,
    DRIVER_STATION_DISCONNECTED,
    BROWNOUT,
    OFF,
    ANIMATION_OFF,
  }

  SystemState systemState = SystemState.OFF;

  public void setDesieredState(DesieredState desieredState) {

    this.desieredState = desieredState;
  }

  public void transitionSystemState(SystemState newsystemState) {

    if (this.systemState == newsystemState) {

      return;
    }
    this.systemState = newsystemState;

    switch (newsystemState) {
      case FULL_LEDS:
        {
          setSolidColor();
        }

      case RAINBOW_ANIMATION:
        {
          setLEDRainAnimationFast();
        }
      case STROBE_ANIMATION:
        {
          setLEDStrobeAnimation(0, 0, 0, 0, m_lastBrownedOutTime, 0, 0);
        }

      case OFF:
        {
          setOffLEDs();
        }
    }
  }

  public LEDs() {

    CANdleConfiguration config = new CANdleConfiguration();
    candle.configAllSettings(config);
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
      case DRIVER_STATION_DISCONNECTED:
        {
          if (Timer.getFPGATimestamp() > m_lastDisconectTime + 5) {
            setAnimationOff();
          }
        }
        break;

      case BROWNOUT:
        {
          if (Timer.getFPGATimestamp() > m_lastBrownedOutTime + 5) {
            setAnimationOff();
          }
        }
        break;
    }
  }
  // if (!DriverStation.isDSAttached())

  // {
  // m_driverstation = Timer.getFPGATimestamp();

  // }

  // if (m_driverstation + 5 >= Timer.getFPGATimestamp()) {
  // setSolidColor();

  // }

  // else {
  // // setOffLEDs();
  // }

  // if (DriverStation.isDSAttached())

  // {
  // m_driverstation = Timer.getFPGATimestamp();

  // }

  // if (m_driverstation + 5 >= Timer.getFPGATimestamp()) {
  // Donothing();
  // }

  public void setLEDRainAnimation() {

    RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.3, 164);
    candle.animate(rainbowAnim);
  }

  public void setLEDRainAnimationFast() {

    RainbowAnimation rAnimation = new RainbowAnimation(1, 1, 164);
    candle.animate(rAnimation);
  }

  public void setAnimationOff() {

    candle.animate(null);
  }

  public void setSolidColor() {
    candle.setLEDs(255, 0, 0, 0, 0, 164);
  }

  public void setOffLEDs() {
    candle.animate(null);
    candle.setLEDs(0, 0, 0, 0, 0, 164);
  }

  public void setLEDStrobeAnimation(
      int r, int g, int b, int w, double speed, int numLed, int ledOffset) {
    StrobeAnimation strobeAnimation = new StrobeAnimation(r, g, b, w, speed, numLed, ledOffset);
    candle.animate(strobeAnimation);
  }

  public void Donothing() {}
}
