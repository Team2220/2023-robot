package frc.robot.commands.Leds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;

public class RainbowLeds extends CommandBase {

  private final LEDs leds;

  public RainbowLeds(LEDs leds) {

    this.leds = leds;

    addRequirements(leds);
  }

  @Override
  public void initialize() {
    leds.setLEDRainAnimation();
    System.out.println("INITIALIZE");
  }

  @Override
  public void end(boolean interrupted) {
    leds.setAnimationOff();
    System.out.println("END");
  }
}
