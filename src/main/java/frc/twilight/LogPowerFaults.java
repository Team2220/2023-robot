package frc.twilight;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.hal.PowerDistributionStickyFaults;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;

public class LogPowerFaults {
    private static boolean firstCheckTalon = true;
    private static boolean firstCheckPdh = true;
    private static PowerDistribution pdh = new PowerDistribution();
    private static ArrayList<Integer> unusedBreakers = new ArrayList<Integer>();
    private static ArrayList<TalonFX> talonFxs = new ArrayList<>();

    public static void add(TalonFX talonFX) {
        talonFxs.add(talonFX);
    }

    public static void checkPDH() {
        if (firstCheckPdh) {
            DataLogManager.log(pdhFaultsToString(pdh.getStickyFaults(), false));
            pdh.clearStickyFaults();
            firstCheckPdh = false;
        }

        PowerDistributionStickyFaults faults = pdh.getStickyFaults();
        String pdhFault = pdhFaultsToString(faults, true);
        if (pdhFault != null) {
            DataLogManager.log(pdhFault);
            pdh.clearStickyFaults();
        }
    }

    public static void addBreakerIgnore(int... num) {
        for (int i : num)
            unusedBreakers.add(i);
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
        if (faults.Channel0BreakerFault && !unusedBreakers.contains(0))
            out += " - Channel 0 Breaker Fault\n";
        if (faults.Channel1BreakerFault && !unusedBreakers.contains(1))
            out += " - Channel 1 Breaker Fault\n";
        if (faults.Channel2BreakerFault && !unusedBreakers.contains(2))
            out += " - Channel 2 Breaker Fault\n";
        if (faults.Channel3BreakerFault && !unusedBreakers.contains(3))
            out += " - Channel 3 Breaker Fault\n";
        if (faults.Channel4BreakerFault && !unusedBreakers.contains(4))
            out += " - Channel 4 Breaker Fault\n";
        if (faults.Channel5BreakerFault && !unusedBreakers.contains(5))
            out += " - Channel 5 Breaker Fault\n";
        if (faults.Channel6BreakerFault && !unusedBreakers.contains(6))
            out += " - Channel 6 Breaker Fault\n";
        if (faults.Channel7BreakerFault && !unusedBreakers.contains(7))
            out += " - Channel 7 Breaker Fault\n";
        if (faults.Channel8BreakerFault && !unusedBreakers.contains(8))
            out += " - Channel 8 Breaker Fault\n";
        if (faults.Channel9BreakerFault && !unusedBreakers.contains(9))
            out += " - Channel 9 Breaker Fault\n";
        if (faults.Channel10BreakerFault && !unusedBreakers.contains(10))
            out += " - Channel 10 Breaker Fault\n";
        if (faults.Channel11BreakerFault && !unusedBreakers.contains(11))
            out += " - Channel 11 Breaker Fault\n";
        if (faults.Channel12BreakerFault && !unusedBreakers.contains(12))
            out += " - Channel 12 Breaker Fault\n";
        if (faults.Channel13BreakerFault && !unusedBreakers.contains(13))
            out += " - Channel 13 Breaker Fault\n";
        if (faults.Channel14BreakerFault && !unusedBreakers.contains(14))
            out += " - Channel 14 Breaker Fault\n";
        if (faults.Channel15BreakerFault && !unusedBreakers.contains(15))
            out += " - Channel 15 Breaker Fault\n";
        if (faults.Channel16BreakerFault && !unusedBreakers.contains(16))
            out += " - Channel 16 Breaker Fault\n";
        if (faults.Channel17BreakerFault && !unusedBreakers.contains(17))
            out += " - Channel 17 Breaker Fault\n";
        if (faults.Channel18BreakerFault && !unusedBreakers.contains(18))
            out += " - Channel 18 Breaker Fault\n";
        if (faults.Channel19BreakerFault && !unusedBreakers.contains(19))
            out += " - Channel 19 Breaker Fault\n";
        if (faults.Channel20BreakerFault && !unusedBreakers.contains(20))
            out += " - Channel 20 Breaker Fault\n";
        if (faults.Channel21BreakerFault && !unusedBreakers.contains(21))
            out += " - Channel 21 Breaker Fault\n";
        if (faults.Channel22BreakerFault && !unusedBreakers.contains(22))
            out += " - Channel 22 Breaker Fault\n";
        if (faults.Channel23BreakerFault && !unusedBreakers.contains(23))
            out += " - Channel 23 Breaker Fault\n";

        if (out == "" && !emptyOnNone)
            out = " - No PDH sticky faults on startup :)";
        else if (out == "" && emptyOnNone)
            return null;

        return "PDH faults:\n" + out;
    }

    // public static void main(String[] args) {
    // if (firstCheck) {
    // DataLogManager.log(checkTalonFX(stickyfaults.hasAnyFault(), false));
    // TalonFX.clearStickyFaults();
    // firstCheck = false;
    // }
    // StickyFaults tFaults = talon.getDeviceID();
    // String talonFault = checkTalonFX(null, tFaults, true);
    // if (talonFault != null) {
    // DataLogManager.log(talonFault);
    // TalonFX.clearStickyFaults();
    // }
    // }
    /* Advanced For Loop */

    public static void checkTalons() {

        StickyFaults talonFaults = new StickyFaults();

            for (TalonFX checkingTalonfxs : talonFxs) {
                checkingTalonfxs.getStickyFaults(talonFaults);
                String nullCheck = checkTalonFX(checkingTalonfxs, talonFaults, true);
                if (nullCheck != null) {
                    DataLogManager.log(nullCheck);
                }

                if (firstCheckTalon)
                    for (TalonFX checkingTalonFxs : talonFxs) {
                        DataLogManager.log(checkTalonFX(checkingTalonFxs, talonFaults, false));
                        firstCheckTalon = true;
                    }
            }
    }

    private static String checkTalonFX(TalonFX talon, StickyFaults faults, boolean emptyOnNone) {

        String out = "";

        if (faults.UnderVoltage)
            out += " - UnderVolyage\n";

        if (faults.ForwardLimitSwitch)
            out += " - ForwardLimitSwitch\n";

        if (faults.ReverseLimitSwitch)
            out += " - ReverseLimitSwitch\n";

        if (faults.ForwardSoftLimit)
            out += " - ForwardSoftLimit\n";

        if (faults.ReverseSoftLimit)
            out += " - ReverseSoftLimit\n";

        if (faults.ResetDuringEn)
            out += " - ResetDuringEn\n";

        if (faults.SensorOverflow)
            out += " - SensorOverflow\n";

        if (faults.SensorOutOfPhase)
            out += " - SensorOutOfPhase\n";

        if (faults.HardwareESDReset)
            out += " - HardwareESDReset\n";

        if (faults.RemoteLossOfSignal)
            out += " - RemoteLossOfSignal\n";

        if (faults.APIError)
            out += " - APIError\n";

        if (faults.SupplyOverV)
            out += " - SupplyOverV\n";

        if (faults.SupplyUnstable)
            out += " - SupplyUnstable\n";

        if (out == "" && !emptyOnNone)
            out = " - No TalonFX " + talon.getDeviceID() + " sticky faults on startup :)";
        else if (out == "" && emptyOnNone)
            return null;

        return "TalonFX" + talon.getDeviceID() + "faults:\n + out";
    }

}
