package frc.robot.commands.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.twilight.tunables.TunableBoolean;

public class ArmPercentOutput extends CommandBase {
  private final Arm m_arm;
  private final DoubleSupplier wristSpeed;
  private final DoubleSupplier shoulderSpeed;

  private static final TunableBoolean disableManual = new TunableBoolean("Manual Control Enabled", true, true, "arm");

  public ArmPercentOutput(DoubleSupplier wristSpeed, DoubleSupplier shoulderSpeed, Arm arm) {
    m_arm = arm;
    addRequirements(arm);
    this.wristSpeed = wristSpeed;
    this.shoulderSpeed = shoulderSpeed;
  }

  @Override
  public void execute() {
    if (!disableManual.getValue()) {
      m_arm.setShoulderPercentOutput(-shoulderSpeed.getAsDouble());
      m_arm.setWristPercentOutput(-wristSpeed.getAsDouble());
    } else {
      m_arm.setShoulderPercentOutput(0);
      m_arm.setWristPercentOutput(0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_arm.holdCurrentPosition();
  }
}
