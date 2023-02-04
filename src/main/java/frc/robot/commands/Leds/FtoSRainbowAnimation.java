package frc.robot.commands.Leds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;

public class FtoSRainbowAnimation extends CommandBase {

  private final LEDs leds;

  public FtoSRainbowAnimation(LEDs leds) {

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
    leds.setLEDRainAnimationFast();
    System.out.println("END");
  }
}
