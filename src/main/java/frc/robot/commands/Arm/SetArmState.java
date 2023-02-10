package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Arm;

public class SetArmState extends CommandBase {
    private final Arm m_arm;
    private final ArmStates armState;

    public SetArmState(ArmStates armState, Arm arm) {
        this.armState = armState;
        m_arm = arm;
        addRequirements(arm);

    }

    @Override
    public void initialize() {
        m_arm.setArmState(armState);
    }
}
