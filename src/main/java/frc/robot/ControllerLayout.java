package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.twilight.Controller;
import frc.twilight.Controller.Button;

public class ControllerLayout {
    public final static Map<Button, Command> DRIVER = Map.ofEntries(
        // Map.entry(Button.A, null),
        // Map.entry(Button.B, null),
        // Map.entry(Button.X, null),
        // Map.entry(Button.Y, null),

        // Bumpers
        // Map.entry(Button.LB, null),
        // Map.entry(Button.RB, null),
        
        // Joystick buttons
        // Map.entry(Button.LS, null),
        // Map.entry(Button.RS, null),

        // Start (right) button
        Map.entry(Button.START, null),

        // Back (left) button
        Map.entry(Button.BACK, null),

        // POV buttons
        Map.entry(Button.UP, null),
        Map.entry(Button.DOWN, null),
        Map.entry(Button.LEFT, null),
        Map.entry(Button.RIGHT, null)
    );


    public final static Map<Button, Command> MANIPULATOR = Map.ofEntries(
        // Map.entry(Button.A, null),
        // Map.entry(Button.B, null),
        // Map.entry(Button.X, null),
        // Map.entry(Button.Y, null),

        // // Bumpers
        // Map.entry(Button.LB, null),
        // Map.entry(Button.RB, null),
        
        // // Joystick buttons
        // Map.entry(Button.LS, null),
        // Map.entry(Button.RS, null),

        // // Start (right) button
        // Map.entry(Button.START, null),

        // // Back (left) button
        // Map.entry(Button.BACK, null),

        // // POV buttons
        // Map.entry(Button.UP, null),
        // Map.entry(Button.DOWN, null),
        // Map.entry(Button.LEFT, null),
        // Map.entry(Button.RIGHT, null)
    );

    public static void mapDriverController(Controller x) {
        for (Button i : DRIVER.keySet()) {
            if (DRIVER.get(i) == null) continue;

            new Trigger(() -> x.getButton(i)).onTrue(DRIVER.get(i));
        }
    }

    public static void mapManipulatorController(Controller x) {
        for (Button i : MANIPULATOR.keySet()) {
            if (MANIPULATOR.get(i) == null) continue;

            new Trigger(() -> x.getButton(i)).onTrue(MANIPULATOR.get(i));
        }
    }
}
