package frc.twilight;

import edu.wpi.first.wpilibj.XboxController;

public class Controller {
    private final XboxController controller;

    private double deadzone = 0.2;

    public Controller(int port) {
      controller = new XboxController(port);
    }

    public double getLeftX() {
      return deadband(controller.getLeftX(), deadzone);
    }

    public double getLeftY() {
      return -deadband(controller.getLeftY(), deadzone);
    }

    public double getRightX() {
      return deadband(controller.getRightX(), deadzone);
    }

    public double getRightY() {
      return -deadband(controller.getRightY(), deadzone);
    }
    
    public void setDeadband(double set) {
      deadzone = set;
    }

    public boolean getButton(Button button) {
      switch(button) {
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

    // Does not work for POV buttons, will return false
    public boolean getButtonPressed(Button button) {
      switch(button) {
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

    // Does not work for POV buttons, will return false
    public boolean getButtonReleased(Button button) {
      switch(button) {
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
