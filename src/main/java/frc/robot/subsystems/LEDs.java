package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import frc.robot.Constants;

public class LEDs {
    CANdle candle = new CANdle(Constants.CANDLE);

    public LEDs() {

        CANdleConfiguration config = new CANdleConfiguration();
        candle.configAllSettings(config);
    }

    public void setLEDAnimation() {

        RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);
        candle.animate(rainbowAnim);
    }

    public void setSolidColor() {
        candle.setLEDs(255, 255, 255);
    }

    public void setLEDStrobeAnimation(int r, int g, int b, int w, double speed, int numLed, int ledOffset) {
        StrobeAnimation strobeAnimation = new StrobeAnimation(r, g, b, w, speed, numLed, ledOffset);
        candle.animate(strobeAnimation);
    }

}
