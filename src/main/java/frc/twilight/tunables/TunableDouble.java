package frc.twilight.tunables;

import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class TunableDouble {
  private double defaultValue;
  private GenericEntry shuffleboard;
  private SimpleWidget shuffleboardWidget;

  /**
   * Creates a TunableDouble. It can be enabled and disabled (Use defaultValue)
   *
   * @param name
   * @param d
   * @param tunable
   */
  public TunableDouble(String name, double d, boolean tunable, String tab) {
    this.defaultValue = d;

    if (tunable) {
      shuffleboardWidget = Shuffleboard.getTab(tab).add(name, d);

      shuffleboard = shuffleboardWidget.getEntry();
    } else {
      shuffleboard = null;
    }
  }

  public TunableDouble(String name, double d, boolean tunable, String tab, DoubleConsumer onChange) {
    this(name, d, tunable, tab);
    addChangeListener(onChange);

  };

  public TunableDouble(String name, double defaultValue, boolean tunable, DoubleConsumer onChange) {
    this(name, defaultValue, tunable);
    addChangeListener(onChange);
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
    if (shuffleboard != null)
      return shuffleboard.getDouble(defaultValue);
    return defaultValue;
  }

  public void addChangeListener(DoubleConsumer onChange) {
    onChange.accept(getValue());
    CommandScheduler.getInstance().getDefaultButtonLoop().bind(
        new Runnable() {
          private double m_oldValue = getValue();

          @Override
          public void run() {
            double newValue = getValue();

            if (m_oldValue != newValue) {
              onChange.accept(newValue);
              m_oldValue = newValue;
            }

          }
        });
  }

}
