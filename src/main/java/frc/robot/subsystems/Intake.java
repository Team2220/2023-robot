package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConfig;

public class Intake extends SubsystemBase {
  private TalonFX intake = new TalonFX(IntakeConfig.INTAKE_TALONFX);

  public Intake() {
    intake.configAllSettings(new TalonFXConfiguration());

    intake.configVoltageCompSaturation(10);

    intake.setInverted(IntakeConfig.INTAKE_INVERTED);

    StatorCurrentLimitConfiguration statorConfig = new StatorCurrentLimitConfiguration();
    statorConfig.enable = true;
    statorConfig.currentLimit = 10;
    intake.configStatorCurrentLimit(statorConfig);

    SupplyCurrentLimitConfiguration supplyConfig = new SupplyCurrentLimitConfiguration();
    supplyConfig.enable = true;
    supplyConfig.currentLimit = 10;
    intake.configSupplyCurrentLimit(supplyConfig);
  }

  public void setPercentOutput(double value) {
    intake.set(TalonFXControlMode.PercentOutput, value);
  }

  public ArrayList<TalonFX> geTalonFXs() {

    ArrayList<TalonFX> musicList = new ArrayList<>();
    musicList.add(intake);
    return musicList;
  }
}
