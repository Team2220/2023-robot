package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.ArmStates;

public class SequentialArmPosition extends CommandBase {
    private final Arm m_arm;
  private final ArmStates armState;

  public SequentialArmPosition(ArmStates armState, Arm arm) {
    this.armState = armState;
    m_arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    // m_arm.setArmState(armState);
    m_arm.setShoulderAngle(armState.shoulderAngle);
  }

  @Override
  public String getName() {
    return "Set " + armState.name();
  }



@Override
public void execute() {
    
}

  @Override
  public boolean isFinished() {
   return true;
  }
}
