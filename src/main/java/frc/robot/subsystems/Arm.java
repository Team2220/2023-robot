package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
    private DutyCycleEncoder wristEncoder = new DutyCycleEncoder(Constants.WRIST_DUTYENCODER);
    private DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(Constants.SHOULDER_DUTYENCODER);

    private TalonFX wrist = new TalonFX(Constants.WRIST_TALONFX);
    private TalonFX shoulder = new TalonFX(Constants.SHOULDER_TALONFX);

    ShuffleboardTab arm = Shuffleboard.getTab("arm");
    GenericEntry shoulderSB = arm.add("shoulder angle", 0).getEntry();
    GenericEntry wristSB = arm.add("wrist angle", 0).getEntry();

    public Arm() {
        wrist.configAllSettings(new TalonFXConfiguration());
        shoulder.configAllSettings(new TalonFXConfiguration());

        wrist.configVoltageCompSaturation(12);
        shoulder.configVoltageCompSaturation(12);

        wrist.setInverted(Constants.WRIST_INVERTED);
        shoulder.setInverted(Constants.SHOULDER_INVERTED);

        SupplyCurrentLimitConfiguration supplyConfig = new SupplyCurrentLimitConfiguration();
        supplyConfig.currentLimit = 33;
        supplyConfig.enable = false;
        shoulder.configSupplyCurrentLimit(supplyConfig);
        wrist.configSupplyCurrentLimit(supplyConfig);

        StatorCurrentLimitConfiguration config = new StatorCurrentLimitConfiguration();
        config.currentLimit = 33;
        config.enable = false;
        shoulder.configStatorCurrentLimit(config);
        wrist.configStatorCurrentLimit(config);

        wrist.config_kP(0, 0);
        wrist.config_kI(0, 0);
        wrist.config_kD(0, 0);

        shoulder.config_kP(0, 0);
        shoulder.config_kI(0, 0);
        shoulder.config_kD(0, 0);

        double wristOffset = getWristPosition() / (Constants.WRIST_GEAR_RATIO) * (Constants.TALONFX_ENCODER_TICKS);
        wrist.setSelectedSensorPosition(wristOffset);

          double shoulderOffset = getShoulderPosition() / (Constants.SHOULDER_GEAR_RATIO) * (Constants.TALONFX_ENCODER_TICKS);
        wrist.setSelectedSensorPosition(shoulderOffset);
    }

    public void setWristPercentOutput(double value) {
        wrist.set(TalonFXControlMode.PercentOutput, value);
    }

    public void setShoulderPercentOutput(double value) {
        shoulder.set(TalonFXControlMode.PercentOutput, value);
    }

    public void setWristAngle(double angle) {
        double gearRatio = Constants.WRIST_GEAR_RATIO;
        double posValue = ((angle/gearRatio)/360) * 2048;
        wrist.set(TalonFXControlMode.Position, posValue);
    }

    public void setShoulderAngle(double angle) {
        double gearRatio = Constants.SHOULDER_GEAR_RATIO;
        double posValue = ((angle/gearRatio)/360) * 2048;
        shoulder.set(TalonFXControlMode.Position, posValue);
    }

    public double getShoulderPosition() {
        return shoulderEncoder.getAbsolutePosition() - Constants.SHOULDER_ENCODER_OFFSET;
    }

    public double getWristPosition() {
        return wristEncoder.getAbsolutePosition() - Constants.WRIST_ENCODER_OFFSET;
    }

    @Override
    public void periodic() {
        shoulderSB.setDouble(getShoulderPosition());
        wristSB.setDouble(getWristPosition());
    }

}
