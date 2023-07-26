package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConfig;
import frc.robot.commands.Arm.SetArmState;
import frc.twilight.tunables.TunableDouble;

class AngularTalonFX {
    private DutyCycleEncoder dutyCycleEncoder;
    private TalonFX talonFX;
    
    private final TunableDouble talonP;
    private final TunableDouble talonI;
    private final TunableDouble talonD;
    private double oldTalonP;
    private double oldTalonI;
    private double oldTalonD;

    public AngularTalonFX(
        int dutyEncoder, int talonId, String name, boolean tunableDoubleEnabled
        ) {
        dutyCycleEncoder = new DutyCycleEncoder(dutyEncoder);
        talonFX = new TalonFX(talonId);

        talonP = new TunableDouble(name + "P", 0.1, tunableDoubleEnabled);
        talonI = new TunableDouble(name + "I", 0, tunableDoubleEnabled);
        talonD = new TunableDouble(name + "D", 0.2, tunableDoubleEnabled);
        oldTalonP = talonP.getValue();
        oldTalonI = talonI.getValue();
        oldTalonD = talonD.getValue();

    }
}