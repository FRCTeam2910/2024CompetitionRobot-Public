// Copyright (c) 2023 FRC 254
// https://github.com/Team254/FRC-2023-Public
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util.drivers;

import java.util.EnumMap;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Phoenix6Util {

    private static boolean configResult = true;

    /**
     * checks the specified error code for issues
     *
     * @param statusCode error code
     * @param message    message to print if error happens
     */
    public static void checkError(StatusCode statusCode, String message) {
        if (statusCode != StatusCode.OK) {
            DriverStation.reportError(message + " " + statusCode, false);
        }
    }

    public static boolean checkErrorAndRetry(Supplier<StatusCode> function, int numTries) {
        StatusCode code = function.get();
        int tries = 0;
        while (code != StatusCode.OK && tries < numTries) {
            DriverStation.reportWarning("Retrying CTRE Device Config " + code.getName(), false);
            code = function.get();
            tries++;
        }
        if (code != StatusCode.OK) {
            DriverStation.reportError(
                    "Failed to execute phoenix 6 api call after " + numTries + " attempts. " + code.getDescription(),
                    false);
            return false;
        }
        return true;
    }

    /**
     * checks the specified error code and throws an exception if there are any issues
     *
     * @param statusCode error code
     * @param message    message to print if error happens
     */
    public static void checkErrorWithThrow(StatusCode statusCode, String message) {
        if (statusCode != StatusCode.OK) {
            throw new RuntimeException(message + " " + statusCode);
        }
    }

    public static boolean checkErrorAndRetry(Supplier<StatusCode> function) {
        return checkErrorAndRetry(function, 5);
    }

    public static boolean applyAndCheckConfiguration(TalonFX talon, TalonFXConfiguration config, int numTries) {
        for (int i = 0; i < numTries; i++) {
            if (checkErrorAndRetry(() -> talon.getConfigurator().apply(config))) {
                // API says we applied config, lets make sure it's right
                if (readAndVerifyConfiguration(talon, config)) {
                    return true;
                } else {
                    DriverStation.reportWarning(
                            "Failed to verify config for talon [" + talon.getDescription() + "] (attempt " + (i + 1)
                                    + " of " + numTries + ")",
                            false);
                }
            } else {
                DriverStation.reportWarning(
                        "Failed to apply config for talon [" + talon.getDescription() + "] (attempt " + (i + 1) + " of "
                                + numTries + ")",
                        false);
            }
        }
        DriverStation.reportError("Failed to apply config for talon after " + numTries + " attempts", false);
        return false;
    }

    public static boolean readAndVerifyConfiguration(TalonFX talon, TalonFXConfiguration config) {
        TalonFXConfiguration readConfig = new TalonFXConfiguration();
        if (!checkErrorAndRetry(() -> talon.getConfigurator().refresh(readConfig))) {
            // could not get config!
            DriverStation.reportWarning("Failed to read config for talon [" + talon.getDescription() + "]", false);
            return false;
        } else if (!TalonFXConfigEquality.isEqual(config, readConfig)) {
            // configs did not match
            DriverStation.reportWarning(
                    "Configuration verification failed for talon [" + talon.getDescription() + "]", false);
            return false;
        } else {
            // configs read and match, Talon OK
            return true;
        }
    }

    public static boolean applyAndCheckConfiguration(TalonFX talon, TalonFXConfiguration config) {
        boolean result = applyAndCheckConfiguration(talon, config, 5);

        //        if (!result) {
        //            LED.getInstance().setConfigureFault(true);
        //        }

        configResult &= result;
        SmartDashboard.putBoolean("Talon Configuration state", configResult);

        return result;
    }

    public enum Fault {
        Hardware,
        OverSupplyV,
        Undervoltage,
        UnstableSupplyV,
        StatorCurrLimit,
        SupplyCurrLimit,
        UnlicensedFeatureInUse,
        BridgeBrownout,
        RemoteSensorReset,
        RemoteSensorPosOverflow,
        RemoteSensorDataInvalid,
        FusedSensorOutOfSync,
        UsingFusedCANcoderWhileUnlicensed,
        MissingDifferentialFX,
        ReverseHardLimit,
        ForwardHardLimit,
        ReverseSoftLimit,
        ForwardSoftLimit,
        ProcTemp,
        DeviceTemp,
    }

    public static void checkFaults(String subsystemName, TalonFX talon) {
        StringBuilder sb = new StringBuilder();
        EnumMap<Fault, Boolean> faults = new EnumMap<>(Fault.class);
        faults.put(Fault.Hardware, talon.getFault_Hardware().getValue());
        faults.put(Fault.OverSupplyV, talon.getFault_OverSupplyV().getValue());
        faults.put(Fault.Undervoltage, talon.getFault_Undervoltage().getValue());
        faults.put(Fault.UnstableSupplyV, talon.getFault_UnstableSupplyV().getValue());
        // faults.put(Fault.StatorCurrLimit, talon.getFault_StatorCurrLimit().getValue());
        // faults.put(Fault.SupplyCurrLimit, talon.getFault_SupplyCurrLimit().getValue());
        faults.put(
                Fault.UnlicensedFeatureInUse,
                talon.getFault_UnlicensedFeatureInUse().getValue());
        faults.put(Fault.BridgeBrownout, talon.getFault_BridgeBrownout().getValue());
        faults.put(Fault.RemoteSensorReset, talon.getFault_RemoteSensorReset().getValue());
        faults.put(
                Fault.RemoteSensorPosOverflow,
                talon.getFault_RemoteSensorPosOverflow().getValue());
        faults.put(
                Fault.RemoteSensorDataInvalid,
                talon.getFault_RemoteSensorDataInvalid().getValue());
        faults.put(
                Fault.FusedSensorOutOfSync,
                talon.getFault_FusedSensorOutOfSync().getValue());
        faults.put(
                Fault.UsingFusedCANcoderWhileUnlicensed,
                talon.getFault_UsingFusedCANcoderWhileUnlicensed().getValue());
        faults.put(
                Fault.MissingDifferentialFX,
                talon.getFault_MissingDifferentialFX().getValue());
        faults.put(Fault.ReverseHardLimit, talon.getFault_ReverseHardLimit().getValue());
        faults.put(Fault.ForwardHardLimit, talon.getFault_ForwardHardLimit().getValue());
        faults.put(Fault.ReverseSoftLimit, talon.getFault_ReverseSoftLimit().getValue());
        faults.put(Fault.ForwardSoftLimit, talon.getFault_ForwardSoftLimit().getValue());
        faults.put(Fault.ProcTemp, talon.getFault_ProcTemp().getValue());
        faults.put(Fault.DeviceTemp, talon.getFault_DeviceTemp().getValue());
        for (var fault : faults.entrySet()) {
            if (fault.getValue()) {
                sb.append(fault.getKey().toString()).append(", ");
            }
        }
        if (!sb.isEmpty()) {
            DriverStation.reportError(subsystemName + ": Talon Faults! " + sb, false);
        }
    }

    public enum StickyFault {
        BootDuringEnable,
        BridgeBrownout,
        DeviceTemp,
        ForwardHardLimit,
        ForwardSoftLimit,
        Hardware,
        OverSupplyV,
        ProcTemp,
        ReverseHardLimit,
        ReverseSoftLimit,
        RemoteSensorReset,
        Undervoltage,
        UnstableSupplyV,
        UnlicensedFeatureInUse
    }

    public static void checkStickyFaults(String subsystemName, TalonFX talon) {
        StringBuilder sb = new StringBuilder();
        EnumMap<StickyFault, Boolean> faults = new EnumMap<>(StickyFault.class);
        faults.put(
                StickyFault.BootDuringEnable,
                talon.getStickyFault_BootDuringEnable().getValue());
        faults.put(
                StickyFault.BridgeBrownout,
                talon.getStickyFault_BridgeBrownout().getValue());
        faults.put(StickyFault.DeviceTemp, talon.getStickyFault_DeviceTemp().getValue());
        faults.put(
                StickyFault.ForwardHardLimit,
                talon.getStickyFault_ForwardHardLimit().getValue());
        faults.put(
                StickyFault.ForwardSoftLimit,
                talon.getStickyFault_ForwardSoftLimit().getValue());
        faults.put(StickyFault.Hardware, talon.getStickyFault_Hardware().getValue());
        faults.put(StickyFault.OverSupplyV, talon.getStickyFault_OverSupplyV().getValue());
        faults.put(StickyFault.ProcTemp, talon.getStickyFault_ProcTemp().getValue());
        faults.put(
                StickyFault.ReverseHardLimit,
                talon.getStickyFault_ReverseHardLimit().getValue());
        faults.put(
                StickyFault.ReverseSoftLimit,
                talon.getStickyFault_ReverseSoftLimit().getValue());
        faults.put(StickyFault.Undervoltage, talon.getStickyFault_Undervoltage().getValue());
        faults.put(
                StickyFault.UnstableSupplyV,
                talon.getStickyFault_UnstableSupplyV().getValue());
        faults.put(
                StickyFault.UnlicensedFeatureInUse,
                talon.getStickyFault_UnlicensedFeatureInUse().getValue());
        faults.put(
                StickyFault.RemoteSensorReset,
                talon.getStickyFault_RemoteSensorReset().getValue());

        for (var fault : faults.entrySet()) {
            if (fault.getValue()) {
                sb.append(fault.getKey().toString()).append(", ");
            }
        }
        if (!sb.isEmpty()) {
            DriverStation.reportError(subsystemName + ": Talon StickyFaults! " + sb, false);
        }

        talon.clearStickyFaults();
    }
}
