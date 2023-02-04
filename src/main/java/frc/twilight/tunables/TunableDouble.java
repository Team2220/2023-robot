package frc.twilight.tunables;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class TunableDouble {
  private double defaultValue;
  private GenericEntry shuffleboard;
  private SimpleWidget shuffleboardWidget;

  /**
   * Creates a TunableDouble. It can be enabled and disabled (Use defaultValue)
   *
   * @param name
   * @param defaultValue
   * @param tunable
   */
  public TunableDouble(String name, double defaultValue, boolean tunable, String tab) {
    this.defaultValue = defaultValue; 

    if (tunable) {
      shuffleboardWidget = Shuffleboard.getTab(tab).add(name, defaultValue);

      shuffleboard = shuffleboardWidget.getEntry();
    } else {
      shuffleboard = null;
    }
  }

  public TunableDouble(String name, double defaultValue, boolean tunable) {
    this(name, defaultValue, tunable, "Tunables");
  }

  public TunableDouble setSpot(int x, int y) {
    if (shuffleboard != null) {
      shuffleboardWidget.withPosition(x, y);
    }

    return this;
  }

  /**
   * @return Value as a double
   */
  public double getValue() {
    if (shuffleboard != null) return shuffleboard.getDouble(defaultValue);
    return defaultValue;
  }
}
