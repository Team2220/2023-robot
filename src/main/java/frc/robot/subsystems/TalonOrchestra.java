package frc.robot.subsystems;

import com.ctre.phoenix.music.Orchestra;
import java.util.ArrayList;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TalonOrchestra extends SubsystemBase {
    Orchestra _orchestra;
    int songSelection = 1;
    boolean playing = false;

    String[] _songs = new String[] {
            // "song1.chrp",
            "song2.chrp",
            "song3.chrp"

    };

    public TalonOrchestra() {
        /** Creates a new ExampleSubsystem. */

        ArrayList<TalonFX> _instruments = new ArrayList<TalonFX>();

        TalonFX talon1 = new TalonFX(1);
        // TalonFX talon2 = new TalonFX(1);
        // TalonFX talon3 = new TalonFX(2);
        // TalonFX talon4 = new TalonFX(3);

        _instruments.add(talon1);
        // _instruments.add(talon2);
        // _instruments.add(talon3);
        // _instruments.add(talon4);

        /* Create the orchestra with the TalonFX instruments */
        _orchestra = new Orchestra(_instruments);
        /*
         * Talon FXs to play music through.
         * More complex music MIDIs will contain several tracks, requiring multiple
         * instruments.
         */
    }
}