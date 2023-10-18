package frc.robot.commands.Arm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmPosition extends CommandBase {
  private final Arm m_arm;
  private final double wristAngle;
  private final double shoulderAngle;

  private final TrapezoidProfile.Constraints constraints = new Constraints(50, 100);
  
  private final TrapezoidProfile.State endWrist;
  private final TrapezoidProfile.State endShoulder;

  private final TrapezoidProfile wrist;
  private final TrapezoidProfile shoulder;

  private double time = 0;

  public ArmPosition(double wristAngle, double shoulderAngle, Arm arm) {
    m_arm = arm;
    addRequirements(arm);
    this.wristAngle = wristAngle;
    this.shoulderAngle = shoulderAngle;

    endWrist = new State(wristAngle, 0);
    endShoulder = new State(shoulderAngle, 0);

    wrist = new TrapezoidProfile(constraints, endWrist, new State(-m_arm.getWristPosition(), 0));
    shoulder = new TrapezoidProfile(constraints, endShoulder, new State(m_arm.getShoulderPosition(), 0));
  }

  @Override
  public void execute() {
    time += 0.02;

    m_arm.setShoulderFromAbsEncoder();
    m_arm.setWristFromAbsEncoder();

    m_arm.setShoulderAngle(shoulder.calculate(time).position);
    m_arm.setWristAngle(wrist.calculate(time).position);
  }

  @Override
  public boolean isFinished() {
    return
      Math.abs(wristAngle - m_arm.getMotorWristPosition()) < 3 &&
      Math.abs(shoulderAngle - m_arm.getMotorShoulderPosition()) < 3;
  }
}
