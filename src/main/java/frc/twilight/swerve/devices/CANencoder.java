package frc.twilight.swerve.devices;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

public class CANencoder implements EncoderBase {
    CANCoder encoder;

    public CANencoder(int canID) {
        encoder = new CANCoder(canID);
        encoder.configFactoryDefault(250);
        encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 250);
        encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360, 250);
    }

    public double getPosition() {
        return encoder.getAbsolutePosition() % 360;
    }
}
