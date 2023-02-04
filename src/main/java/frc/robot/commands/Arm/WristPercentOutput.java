package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class WristPercentOutput extends CommandBase {
  private final Arm m_arm;
  private final double speed;

  public WristPercentOutput(double speed, Arm arm) {
    m_arm = arm;
    addRequirements(arm);
    this.speed = speed;
  }

  @Override
  public void execute() {
    m_arm.setWristPercentOutput(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.setWristPercentOutput(0);
  }
}
