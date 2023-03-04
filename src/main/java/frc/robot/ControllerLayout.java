package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.twilight.Controller.Button;

public class ControllerLayout {
    public final static Map<Button, Command> DRIVER = Map.ofEntries(
        Map.entry(Button.A, null),
        Map.entry(Button.B, null),
        Map.entry(Button.X, null),
        Map.entry(Button.Y, null)
    );


    public final static Map<Button, Command> MANIPULATOR = Map.ofEntries(
        Map.entry(Button.A, null),
        Map.entry(Button.B, null),
        Map.entry(Button.X, null),
        Map.entry(Button.Y, null)
    );
}
