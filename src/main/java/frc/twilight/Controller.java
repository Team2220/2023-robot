package frc.twilight;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

/**
 * Controller class for Xbox controllers on the FRC robot.
 * 
 * This class provides an easy-to-use interface for interacting with an Xbox
 * controller
 * connected to the FRC robot. It provides methods for getting joystick values,
 * button states,
 * and button events (presses and releases). It also allows for setting a
 * deadband for joystick
 * values to eliminate noise.
 * 
 * @author TeamTwilight
 */
public class Controller {
  private final XboxController controller;

  /**
   * The deadband value to use for joystick values.
   * The default value is 0.2.
   */
  private double deadzone = 0.2;

  /**
   * Creates a new instance of the Controller class.
   * 
   * @param port The USB port number the controller is connected to.
   */
  public Controller(int port) {
    controller = new XboxController(port);
  }

  /**
   * Gets the left X-axis joystick value.
   * 
   * @return the left X-axis joystick value, with deadband applied.
   */
  public double getLeftX() {
    return deadband(controller.getLeftX(), deadzone);
  }

  /**
   * Gets the left Y-axis joystick value.
   * 
   * @return the left Y-axis joystick value, with deadband applied.
   */
  public double getLeftY() {
    return -deadband(controller.getLeftY(), deadzone);
  }

  /**
   * Gets the right X-axis joystick value.
   * 
   * @return the right X-axis joystick value, with deadband applied.
   */
  public double getRightX() {
    return deadband(controller.getRightX(), deadzone);
  }

  /**
   * Gets the right Y-axis joystick value.
   * 
   * @return the right Y-axis joystick value, with deadband applied.
   */
  public double getRightY() {
    return -deadband(controller.getRightY(), deadzone);
  }

  /**
   * Sets the deadband value to use for joystick values.
   * 
   * @param set the deadband value to use.
   */
  public void setDeadband(double set) {
    deadzone = set;
  }

  /**
   * Gets the state of a button.
   * 
   * @param button the button to check
   * @return true if the button is pressed, false otherwise
   */
  public boolean getButton(Button button) {
    switch (button) {
      case A:
        return controller.getAButton();
      case B:
        return controller.getBButton();
      case X:
        return controller.getXButton();
      case Y:
        return controller.getYButton();
      case LB:
        return controller.getLeftBumper();
      case RB:
        return controller.getRightBumper();
      case BACK:
        return controller.getBackButton();
      case START:
        return controller.getStartButton();
      case UP:
        return controller.getPOV() == 0;
      case DOWN:
        return controller.getPOV() == 180;
      case LEFT:
        return controller.getPOV() == 270;
      case RIGHT:
        return controller.getPOV() == 90;
      case LS:
        return controller.getLeftStickButton();
      case RS:
        return controller.getRightStickButton();
      default:
        return false;
    }
  }

  /**
   * Gets the state of a button press event.
   * Note: does not work for POV buttons, will return false.
   * 
   * @param button the button to check
   * @return true if the button was just pressed, false otherwise
   */
  public boolean getButtonPressed(Button button) {
    switch (button) {
      case A:
        return controller.getAButtonPressed();
      case B:
        return controller.getBButtonPressed();
      case X:
        return controller.getXButtonPressed();
      case Y:
        return controller.getYButtonPressed();
      case LB:
        return controller.getLeftBumperPressed();
      case RB:
        return controller.getRightBumperPressed();
      case BACK:
        return controller.getBackButtonPressed();
      case START:
        return controller.getStartButtonPressed();
      case LS:
        return controller.getLeftStickButtonPressed();
      case RS:
        return controller.getRightStickButtonPressed();
      default:
        return false;
    }
  }

  /**
   * Gets the state of a button release event.
   * Note: does not work for POV buttons, will return false.
   * 
   * @param button the button to check
   * @return true if the button was just released, false otherwise
   */
  public boolean getButtonReleased(Button button) {
    switch (button) {
      case A:
        return controller.getAButtonReleased();
      case B:
        return controller.getBButtonReleased();
      case X:
        return controller.getXButtonReleased();
      case Y:
        return controller.getYButtonReleased();
      case LB:
        return controller.getLeftBumperReleased();
      case RB:
        return controller.getRightBumperReleased();
      case BACK:
        return controller.getBackButtonReleased();
      case START:
        return controller.getStartButtonReleased();
      case LS:
        return controller.getLeftStickButtonReleased();
      case RS:
        return controller.getRightStickButtonReleased();
      default:
        return false;
    }
  }

  /**
   * Applies the deadband to a joystick value.
   * 
   * @param value    the joystick value to apply the deadband to
   * @param deadband the deadband value to use
   * @return the joystick value with the deadband applied
   */
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }
/**
 * make it ✧･ﾟ: *✧･ﾟ:*vibrate*:･ﾟ✧*:･ﾟ✧ babey
 */
  public void runRumble (){
    controller.setRumble(RumbleType.kLeftRumble, deadzone);
    controller.setRumble(RumbleType.kRightRumble, deadzone);
    /**
     * if(arm all the way || the other way)
     * then (vibrate)
     * 
     */
  }

  /**
   * Enumeration of all buttons on an Xbox controller.
   */
  public enum Button {
    A,
    B,
    X,
    Y,
    LB,
    RB,
    BACK,
    START,
    UP,
    DOWN,
    LEFT,
    RIGHT,
    LS,
    RS
  }
}
