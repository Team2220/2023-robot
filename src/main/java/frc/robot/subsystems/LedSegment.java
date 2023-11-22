package frc.robot.subsystems;

import java.util.function.Consumer;

import com.ctre.phoenix.led.Animation;

public class LedSegment {
    Consumer<Animation> setAnimation;
    boolean hasBeenSet;

    public LedSegment(Consumer<Animation> setAnimation, boolean hasBeenSet) {
        this.setAnimation = setAnimation;
        this.hasBeenSet = hasBeenSet;
    }

    public boolean reset() {
        return hasBeenSet = false;
    }

    public void setAnimationIfAble(Animation animation) {
        if (!hasBeenSet) {
            setAnimation.accept(animation);
            hasBeenSet = true;
        }
    }
}
