package frc.robot.subsystems;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConfig;
import frc.twilight.tunables.TunableDouble;

public class Intake extends SubsystemBase {
  private TalonFX intake = new TalonFX(IntakeConfig.INTAKE_TALONFX);

  private TunableDouble intakeP = new TunableDouble("IntakeP", 0, true); 
  private TunableDouble intakeI = new TunableDouble("IntakeI", 0, true);
  private TunableDouble intakeD = new TunableDouble("IntakeD", 0, true);

  private double oldP = intakeP.getValue();
  private double oldI = intakeI.getValue();
  private double oldD = intakeD.getValue();

  private double setPos = 0;

  public Intake() {
    intake.configAllSettings(new TalonFXConfiguration());

    intake.configVoltageCompSaturation(10);

    intake.setInverted(IntakeConfig.INTAKE_INVERTED);
    intake.setNeutralMode(NeutralMode.Brake);

    intake.config_kP(0, oldP);
    intake.config_kI(0, oldI);
    intake.config_kD(0, oldD);

    StatorCurrentLimitConfiguration statorConfig = new StatorCurrentLimitConfiguration();
    statorConfig.enable = false;
    statorConfig.currentLimit = 10;
    intake.configStatorCurrentLimit(statorConfig);

    SupplyCurrentLimitConfiguration supplyConfig = new SupplyCurrentLimitConfiguration();
    supplyConfig.enable = false;
    supplyConfig.currentLimit = 10;
    intake.configSupplyCurrentLimit(supplyConfig);
  }

  public void setPercentOutput(double value) {
    if (intakeP.getValue() != oldP) {
      intake.config_kP(0, intakeP.getValue());
      oldP = intakeP.getValue();
    }
  
    if (intakeI.getValue() != oldI) {
      intake.config_kI(0, intakeI.getValue());
      oldI = intakeI.getValue();
    }
  
    if (intakeD.getValue() != oldD) {
      intake.config_kD(0, intakeD.getValue());
      oldD = intakeD.getValue();
    }

    if (value == 0) {
      if (setPos == 0) 
        setPos = intake.getSelectedSensorPosition();

      intake.set(TalonFXControlMode.Position, setPos);

      return;
    }

    setPos = 0;

    intake.set(TalonFXControlMode.PercentOutput, value);
  }

  public ArrayList<TalonFX> geTalonFXs() {

    ArrayList<TalonFX> musicList = new ArrayList<>();
    musicList.add(intake);
    return musicList;
  }
}
