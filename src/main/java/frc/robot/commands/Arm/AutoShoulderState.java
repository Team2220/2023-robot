package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm.ArmStates;
import frc.robot.subsystems.Arm;

public class AutoShoulderState extends CommandBase {
  private final Arm m_arm;
  private final ArmStates armState;

  public AutoShoulderState(ArmStates armState, Arm arm) {
    this.armState = armState;
    m_arm = arm;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    m_arm.setShoulderAngle(armState.shoulderAngle);
  }

  @Override
  public String getName() {
    return "Set " + armState.name();
  }

  @Override
  public boolean isFinished() {
   return Math.abs(armState.shoulderAngle - m_arm.getMotorShoulderPosition()) < 3;
  }
}