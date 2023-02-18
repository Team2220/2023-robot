package frc.robot;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.networktables.NTSendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class CommandChooser implements AutoCloseable, NTSendable {

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public void addOption(Command command) {
    autoChooser.addOption(command.getName(), command);
  }

  public void setDefaultOption(Command command) {
    autoChooser.setDefaultOption(command.getName(), command);
  }

  public Command getSelected() {
    return autoChooser.getSelected();
  }

  @Override
  public void close() throws Exception {
    autoChooser.close();
  }

  @Override
  public void initSendable(NTSendableBuilder builder) {
    autoChooser.initSendable(builder);
  }
}