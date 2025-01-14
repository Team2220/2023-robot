package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.EventLoop;

public final class Fault {
    private boolean isActive = false;
    private boolean wasActive = false;
    private Alert errorAlert;
    private Alert warningAlert;
    private BooleanLogEntry booleanLog;

    public Fault(String description) {
        this.errorAlert = new Alert("[Active] " + description, Alert.AlertType.ERROR);
        this.warningAlert = new Alert("[Recent] " + description, Alert.AlertType.WARNING);
        this.booleanLog = new BooleanLogEntry(DataLogManager.getLog(), description);
        FaultRegistry.register(this);
    }

    public void setIsActive(boolean isActive) {
        this.isActive = isActive;
        booleanLog.append(isActive);

        if (isActive) {
            wasActive = true;
            errorAlert.set(true);
        } else {
            errorAlert.set(false);
            if (wasActive) {
                warningAlert.set(true);
            }
        }
    }

    public boolean isActive() {
        return isActive;
    }

    public boolean wasActive() {
        return wasActive;
    }

    public static void autoUpdating(String description, EventLoop eventLoop, BooleanSupplier booleanSupplier) {
        Fault fault = new Fault(description);
        eventLoop.bind(() -> {
            fault.setIsActive(booleanSupplier.getAsBoolean());
        });
    }

    public static void autoUpdating(String description, BooleanSupplier booleanSupplier) {
        autoUpdating(description, EventLoops.oncePerSec, booleanSupplier);
    }

    public static void setupDefaultFaults() {
        autoUpdating("canBusUtilization", () -> {
            return RobotController.getCANStatus().percentBusUtilization > 0.8;
        });
    }
}
