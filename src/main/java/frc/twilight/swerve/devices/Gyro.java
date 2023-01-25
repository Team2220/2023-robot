package frc.twilight.swerve.devices;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import frc.twilight.swerve.config.GeneralConfig;

public class Gyro {
    private final AHRS ahrs;

    public Gyro() {
        ahrs = new AHRS(SPI.Port.kMXP);
        setPosition(GeneralConfig.DT_START_GYRO);
    }

    public double getAngle() {
        return ahrs.getAngle() % 360;
    }

    public double getAngleSpeed() {
        return ahrs.getRate();
    }

    public void setPosition(double offset) {
        ahrs.reset();
        ahrs.setAngleAdjustment(-offset);
    }

    public void zeroSensor() {
        ahrs.reset();
        ahrs.setAngleAdjustment(0);
    }
}
