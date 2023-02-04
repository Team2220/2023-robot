package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase{
    CANdle candle = new CANdle(Constants.CANDLE);

    public LEDs() {

        CANdleConfiguration config = new CANdleConfiguration();
        candle.configAllSettings(config);
    }

    public void setLEDRainAnimation() {

        RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.3, 164);
        candle.animate(rainbowAnim);
    }

public void setLEDRainAnimationFast(){

    RainbowAnimation rAnimation = new RainbowAnimation(1, 1, 164);
candle.animate(rAnimation);
}

public void setAnimationOff(){

candle.animate(null);


}

    public void setSolidColor() {
        candle.setLEDs(255, 0, 0, 0, 0, 164);
    }

    
    public void setOffLEDs() {
        candle.setLEDs(0, 0, 0);
    }

    public void setLEDStrobeAnimation(int r, int g, int b, int w, double speed, int numLed, int ledOffset) {
        StrobeAnimation strobeAnimation = new StrobeAnimation(r, g, b, w, speed, numLed, ledOffset);
        candle.animate(strobeAnimation);
    }

}
