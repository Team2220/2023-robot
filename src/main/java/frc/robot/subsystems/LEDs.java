package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  private CANdle left = new CANdle(Constants.LEDS.LEFT);
  private CANdle right = new CANdle(Constants.LEDS.RIGHT);

  
  public LEDs(LedSegment[] ledSeg, LedSignal[] sigs) {
  
  }
  @Override
  public void periodic() {
    
  }
}
