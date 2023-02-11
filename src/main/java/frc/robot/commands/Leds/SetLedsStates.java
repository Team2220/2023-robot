package frc.robot.commands.Leds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.LEDs.DesieredState;

public class SetLedsStates extends CommandBase {
  private final LEDs m_LEDs;
  private final DesieredState m_DesieredState;

  public SetLedsStates(DesieredState desieredState, LEDs leds) {
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

}
