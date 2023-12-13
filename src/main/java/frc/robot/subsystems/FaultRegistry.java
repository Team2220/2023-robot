package frc.robot.subsystems;

import java.util.ArrayList;

public class FaultRegistry {
    private static ArrayList<Fault> faults = new ArrayList<>(); 
    public static boolean hasAnyActive() {
        for (Fault fault : faults) {
            if (fault.isActive()) {
                return true;
            }
        }
        return false;
    }

    public static void register(Fault fault) {
        faults.add(fault);
    }
}
