package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class WristPercentOutput extends CommandBase {
  private final Arm m_arm;
  private final DoubleSupplier speed;

  public WristPercentOutput(DoubleSupplier speed, Arm arm) {
    m_arm = arm;
    addRequirements(arm);
    this.speed = speed;
  }

  @Override
  public void execute() {
    m_arm.setWristPercentOutput(speed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.setWristPercentOutput(0);
  }
}
