package frc.twilight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase{
    private NetworkTable table;

    public Limelight(String tableName) {
        table = NetworkTableInstance.getDefault().getTable(tableName);
    }

    public void periodic(){
        //prints april tag
        System.out.println(aprilTag());
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
    public Double[] CameraTransform() {
        return table.getEntry("camtran").getDoubleArray(new Double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
     }
     public Double[] RobotTransform() {
        return table.getEntry("botpose").getDoubleArray(new Double[] {0.0, .0, 0.0, 0.0, 0.0, 0.0});
     }
}
