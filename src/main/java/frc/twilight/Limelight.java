package frc.twilight;

import java.lang.reflect.Constructor;
import java.util.function.Function;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable table;

    public Limelight(String tableName) {
        table = NetworkTableInstance.getDefault().getTable(tableName);
    }

    public boolean seesTarget() {
        long tv = table.getEntry("tv").getInteger(0);
        if (tv == 1) {
            return true;
        } else {
            return false;
        }
    }

    public long aprilTag() {
        return table.getEntry("tid").getInteger(0);
    }

    public long MCU() {
       return table.getEntry("thor").getInteger(0);
    }
    public long CameraTransform() {
        return table.getEntry("camtran").getInteger(0);
     }
     public Double[] RobotTransform() {
        int[] getDoubleArray = new int[] {0, 0, 0, 0, 0, 0};
        return table.getEntry("botpose").getDoubleArray(new Double[] {9.0, 8.0, 0.0});
     }
}
