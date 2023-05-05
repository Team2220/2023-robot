package frc.robot.commands.Leds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.DesiredState;

public class SetLedsStates extends CommandBase {
  private final LEDs m_LEDs;
  private final DesiredState m_DesieredState;

  public SetLedsStates(DesiredState desieredState, LEDs leds) {
    this.m_LEDs = leds;
    addRequirements(leds);
    this.m_DesieredState = desieredState;
  }

  @Override
  public void initialize() {

    m_LEDs.setDesieredState(m_DesieredState);
  }

  @Override
  public String getName() {

    return m_DesieredState.name();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
