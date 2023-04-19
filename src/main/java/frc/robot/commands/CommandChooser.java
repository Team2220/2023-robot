package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandChooser  {

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private Command defaultAuto;

  public void addOption(Command command) {
    autoChooser.addOption(command.getName(), command);
  }

  public void setDefaultOption(Command command) {
    autoChooser.setDefaultOption(command.getName(), command);
    defaultAuto = command;
  }

  public boolean isDefaultSelected() {
    return (autoChooser.getSelected() == defaultAuto);
  }

  public Command getSelected() {
    return autoChooser.getSelected();
  }
  public SendableChooser<Command> getSendableChooser(){
    return autoChooser;
  }
}
