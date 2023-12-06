package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;

public class LedSignal {
    String name;
    BooleanSupplier isActive;
    Animation animation;
    double debounce;
    private LedSegment[] segments;

    public LedSignal(String name, BooleanSupplier isActive, Animation animation, double debounce,
            LedSegment[] segments) {
        this.name = name;
        this.isActive = isActive;
        this.animation = animation;
        this.debounce = debounce;
        this.segments = segments;
    }

    public LedSignal(String name, BooleanSupplier isActive, Animation animation, double debounce) {
        this(name, isActive, animation, debounce, new LedSegment[] {});
    }

    public LedSignal(String name2, BooleanSupplier isActive2, StrobeAnimation strobeAnimation, int debounce2, boolean b) {
}
public void update(LedSegment[] allSegments) {
        if (isActive.getAsBoolean() == true) {
            if (segments.length == 0) {
                for (LedSegment segment : allSegments) {
                    segment.setAnimationIfAble(animation);
                }
            } else {
                for (LedSegment segment : segments) {
                    segment.setAnimationIfAble(animation);
                }
            }
        }
    }

    // starter signals
    public static LedSignal isDSConnected() {
        // fade blue
        SingleFadeAnimation singleFadeAnimation = new SingleFadeAnimation(0, 0, 100, 0, .5, 164);
        return new LedSignal("isDSConnected", DriverStation::isDSAttached, singleFadeAnimation, 0);
    }

    public static LedSignal isBrownedOut() {
        // blink red
        StrobeAnimation strobeAnimation = new StrobeAnimation(64, 0, 0, 0, 0.1, 164);
        return new LedSignal("isBrownedOut", RobotController::isBrownedOut, strobeAnimation, 0);
    }

    public static void hasActiveFault() {
        // blink orange
        StrobeAnimation strobeAnimation = new StrobeAnimation(246, 147, 0, 0, 0.1, 164);
        // return new LedSignal("hasActiveFault", RobotController::isBrownedOut, strobeAnimation, 0);
    }

    public static LedSignal isEndGame() {
        // blink yellow
        StrobeAnimation strobeAnimation = new StrobeAnimation(246, 247, 0, 0, 0.1, 164);
        return new LedSignal("isEndGame", () -> {
            return DriverStation.getMatchTime() <= 15;
        }, strobeAnimation, 0);
    }

    public static void hasTarget() {
        // private final CANdle left = new CANdle(Constants.LEDS.LEFT);
        // private final CANdle right = new CANdle(Constants.LEDS.RIGHT);
        // solid reen as long as target present
        // left.setLEDs(0, 0, 225, 0, 0, 164);
        // right.setLEDs(0, 0, 225, 0, 0, 164);
    }
}
