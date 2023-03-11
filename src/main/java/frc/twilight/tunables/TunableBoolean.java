package frc.twilight.tunables;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;

public class TunableBoolean {
  private boolean defaultValue;
  private GenericEntry shuffleboard;
  private SimpleWidget shuffleboardWidget;

  /**
   * Creates a TunableDouble. It can be enabled and disabled (Use defaultValue)
   *
   * @param name
   * @param defaultValue
   * @param tunable
   */
  public TunableBoolean(String name, boolean defaultValue, boolean tunable, String tab) {
    this.defaultValue = defaultValue;

    if (tunable) {
      shuffleboardWidget = Shuffleboard.getTab(tab).add(name, defaultValue).withWidget(BuiltInWidgets.kToggleSwitch);

      shuffleboard = shuffleboardWidget.getEntry();
    } else {
      shuffleboard = null;
    }
  }

  public TunableBoolean(String name, boolean defaultValue, boolean tunable) {
    this(name, defaultValue, tunable, "Tunables");
  }

  public TunableBoolean setSpot(int x, int y) {
    if (shuffleboard != null) {
      shuffleboardWidget.withPosition(x, y);
    }

    return this;
  }

  /**
   * @return Value as a double
   */
  public boolean getValue() {
    if (shuffleboard != null) return shuffleboard.getBoolean(defaultValue);
    return defaultValue;
  }
}
