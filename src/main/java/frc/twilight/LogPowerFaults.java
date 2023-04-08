package frc.twilight;

import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;

public class LogPowerFaults {
    private static boolean firstCheck = true;
    private static PowerDistribution pdh = new PowerDistribution();

    public static void check() {
        if (firstCheck) {
            DataLogManager.log(pdhFaultsToString(pdh.getStickyFaults(), false));
            pdh.clearStickyFaults();
            firstCheck = false;
        }

        PowerDistributionStickyFaults faults = pdh.getStickyFaults();
        String pdhFault = pdhFaultsToString(faults, true);
        if (pdhFault != null) {
            DataLogManager.log(pdhFault);
            pdh.clearStickyFaults();
        }
    }

    public static String pdhFaultsToString(PowerDistributionStickyFaults faults, boolean emptyOnNone) {
        String out = "";

        if (faults.Brownout)
            out += " - Brownout\n";
        if (faults.HasReset)
            out += " - Reset\n";
        if (faults.CanWarning)
            out += " - CAN Warning\n";
        if (faults.CanBusOff)
            out += " - CAN Bus Off\n";
        if (faults.Channel0BreakerFault)
            out += " - Channel 0 Breaker Fault\n";
        if (faults.Channel1BreakerFault)
            out += " - Channel 1 Breaker Fault\n";
        if (faults.Channel2BreakerFault)
            out += " - Channel 2 Breaker Fault\n";
        if (faults.Channel3BreakerFault)
            out += " - Channel 3 Breaker Fault\n";
        if (faults.Channel4BreakerFault)
            out += " - Channel 4 Breaker Fault\n";
        if (faults.Channel5BreakerFault)
            out += " - Channel 5 Breaker Fault\n";
        if (faults.Channel6BreakerFault)
            out += " - Channel 6 Breaker Fault\n";
        if (faults.Channel7BreakerFault)
            out += " - Channel 7 Breaker Fault\n";
        if (faults.Channel8BreakerFault)
            out += " - Channel 8 Breaker Fault\n";
        if (faults.Channel9BreakerFault)
            out += " - Channel 9 Breaker Fault\n";
        if (faults.Channel10BreakerFault)
            out += " - Channel 10 Breaker Fault\n";
        if (faults.Channel11BreakerFault)
            out += " - Channel 11 Breaker Fault\n";
        if (faults.Channel12BreakerFault)
            out += " - Channel 12 Breaker Fault\n";
        if (faults.Channel13BreakerFault)
            out += " - Channel 13 Breaker Fault\n";
        if (faults.Channel14BreakerFault)
            out += " - Channel 14 Breaker Fault\n";
        if (faults.Channel15BreakerFault)
            out += " - Channel 15 Breaker Fault\n";
        if (faults.Channel16BreakerFault)
            out += " - Channel 16 Breaker Fault\n";
        if (faults.Channel17BreakerFault)
            out += " - Channel 17 Breaker Fault\n";
        if (faults.Channel18BreakerFault)
            out += " - Channel 18 Breaker Fault\n";
        if (faults.Channel19BreakerFault)
            out += " - Channel 19 Breaker Fault\n";
        if (faults.Channel20BreakerFault)
            out += " - Channel 20 Breaker Fault\n";
        if (faults.Channel21BreakerFault)
            out += " - Channel 21 Breaker Fault\n";
        if (faults.Channel22BreakerFault)
            out += " - Channel 22 Breaker Fault\n";
        if (faults.Channel23BreakerFault)
            out += " - Channel 23 Breaker Fault\n";

        if (out == "" && !emptyOnNone)
            out = " - No PDH sticky faults on startup :)";
        else if (out == "" && emptyOnNone)
            return null;

        return "PDH faults:\n" + out;
    }
}
