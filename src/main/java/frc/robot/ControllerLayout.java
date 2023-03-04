package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.twilight.Controller.Button;

public class ControllerLayout {
    public final static Map<Button, Command> DRIVER = Map.ofEntries(
        Map.entry(Button.A, null),
        Map.entry(Button.B, null),
        Map.entry(Button.X, null),
        Map.entry(Button.Y, null),

        // Bumpers
        Map.entry(Button.LB, null),
        Map.entry(Button.RB, null),
        
        // Joystick buttons
        Map.entry(Button.LS, null),
        Map.entry(Button.RS, null),

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
        Map.entry(Button.A, null),
        Map.entry(Button.B, null),
        Map.entry(Button.X, null),
        Map.entry(Button.Y, null),

        // Bumpers
        Map.entry(Button.LB, null),
        Map.entry(Button.RB, null),
        
        // Joystick buttons
        Map.entry(Button.LS, null),
        Map.entry(Button.RS, null),

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
}
