package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmPercentOutput extends CommandBase {
  private final Arm m_arm;
  private final DoubleSupplier wristSpeed;
  private final DoubleSupplier shoulderSpeed;

  public ArmPercentOutput(DoubleSupplier wristSpeed, DoubleSupplier shoulderSpeed, Arm arm) {
    m_arm = arm;
    addRequirements(arm);
    this.wristSpeed = wristSpeed;
    this.shoulderSpeed = shoulderSpeed;
  }

  @Override
  public void execute() {
    m_arm.setShoulderPercentOutput(-shoulderSpeed.getAsDouble());
    m_arm.setWristPercentOutput(-wristSpeed.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.holdCurrentPosition();
  }
}
