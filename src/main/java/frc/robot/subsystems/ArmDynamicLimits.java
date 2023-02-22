package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmDynamicLimits extends SubsystemBase {
    static InterpolatingTreeMap<Double, Double> map = new InterpolatingTreeMap<>();

    static {
        map.put(45.0, -135.0);
        map.put(90.0, -90.0);
        map.put(135.0, -20.0);
        map.put(171.0, 10.0);
    }

    public class WristLimits implements Interpolatable<WristLimits> {
        private double upperLimit;
        private double lowerLimit;

        public WristLimits(double upperLimit, double lowerLimit) {
            this.upperLimit = upperLimit;
            this.lowerLimit = lowerLimit;
        }

        @Override
        public WristLimits interpolate(WristLimits endValue, double t) {
            return new WristLimits(
                MathUtil.interpolate(upperLimit, endValue.upperLimit, t),
                MathUtil.interpolate(lowerLimit, endValue.lowerLimit, t));
        }
    }
}
