package frc.robot.commands.Leds;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDs;
public class FullLeds extends CommandBase {

    private final LEDs leds;

    public FullLeds(LEDs leds) {

        this.leds = leds;

        addRequirements(leds);

    }
    @Override
    public void initialize() {
        leds.setSolidColor();
        System.out.println("INITIALIZE");
    }

    
    @Override
    public void end(boolean interrupted) {
        leds.setOffLEDs();
        System.out.println("END");
    }
    
}
