package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX left = new TalonFX(Constants.INTAKE_LEFT_TALONFX);
    private TalonFX right = new TalonFX(Constants.INTAKE_RIGHT_TALONFX);

    public Intake(){
    left.configAllSettings(new TalonFXConfiguration());
    right.configAllSettings(new TalonFXConfiguration());


        left.configVoltageCompSaturation(10);
        right.configVoltageCompSaturation(10);

        left.setInverted(Constants.INTAKE_LEFT_INVERTED);
        right.setInverted(Constants.INTAKE_RIGHT_INVERTED);
        
        StatorCurrentLimitConfiguration statorConfig = new StatorCurrentLimitConfiguration();
        statorConfig.enable = true;
        statorConfig.currentLimit = 10;
        right.configStatorCurrentLimit(statorConfig);
        left.configStatorCurrentLimit(statorConfig);     
        
        SupplyCurrentLimitConfiguration supplyConfig = new SupplyCurrentLimitConfiguration();
        supplyConfig.enable = true;
        supplyConfig.currentLimit = 10;
        right.configSupplyCurrentLimit(supplyConfig);
        left.configSupplyCurrentLimit(supplyConfig);

        right.follow(left);
    }

    public void setPercentOutput(double value) {
        right.set(TalonFXControlMode.PercentOutput, value);
    }
    public ArrayList<TalonFX> geTalonFXs() {

        ArrayList<TalonFX> musicList = new ArrayList<>();
        musicList.add(left);
        musicList.add(right);
        return musicList;
    }
}
