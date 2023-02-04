package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmPosition extends CommandBase {
  private final Arm m_arm;
  private final double wristAngle;
  private final double shoulderAngle;

  public ArmPosition(double wristAngle, double shoulderAngle, Arm arm) {
    m_arm = arm;
    addRequirements(arm);
    this.wristAngle = wristAngle;
    this.shoulderAngle = shoulderAngle;
  }

  @Override
  public void execute() {
    m_arm.setShoulderAngle(shoulderAngle);
    m_arm.setWristAngle(wristAngle);
  }
}
