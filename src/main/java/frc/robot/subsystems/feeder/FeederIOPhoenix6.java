package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.config.PortConfiguration;
import frc.robot.util.drivers.Phoenix6Util;
import frc.robot.util.drivers.TalonFXFactory;

public class FeederIOPhoenix6 implements FeederIO {
    private final TalonFX teacupMotor;
    private final TalonFX feederRight;
    private final TalonFX feederLeft;
    private final DigitalInput beamBreakSensor;

    private final VoltageOut teacupRequest = new VoltageOut(0.0, true, false, false, false);
    private final VoltageOut slurpRequest = new VoltageOut(0.0, true, false, false, false);

    public FeederIOPhoenix6(PortConfiguration ports) {
        teacupMotor = TalonFXFactory.createDefaultTalon(ports.teacupMotorCanDeviceID);
        feederRight = TalonFXFactory.createDefaultTalon(ports.rightFeederMotorID);
        feederLeft = TalonFXFactory.createDefaultTalon(ports.leftFeederMotorID);
        beamBreakSensor = new DigitalInput(ports.beamBreakDIOId);

        TalonFXConfiguration teacupConfig = new TalonFXConfiguration();
        teacupConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        teacupConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        teacupConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        teacupConfig.CurrentLimits.SupplyCurrentLimit = 80;
        teacupConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        teacupConfig.CurrentLimits.StatorCurrentLimit = 200;
        teacupConfig.Voltage.PeakForwardVoltage = 12.0;
        teacupConfig.Voltage.PeakReverseVoltage = -12.0;
        teacupConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

        TalonFXConfiguration slurpConfig = new TalonFXConfiguration();
        slurpConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        slurpConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        slurpConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        slurpConfig.CurrentLimits.SupplyCurrentLimit = 80;
        slurpConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        slurpConfig.CurrentLimits.StatorCurrentLimit = 200;
        slurpConfig.Voltage.PeakForwardVoltage = 12.0;
        slurpConfig.Voltage.PeakReverseVoltage = -12.0;
        slurpConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

        Phoenix6Util.applyAndCheckConfiguration(teacupMotor, teacupConfig);
        Phoenix6Util.applyAndCheckConfiguration(feederRight, slurpConfig);
        Phoenix6Util.applyAndCheckConfiguration(feederLeft, slurpConfig);
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.teacupMotorConnected = BaseStatusSignal.refreshAll(
                        teacupMotor.getMotorVoltage(), teacupMotor.getSupplyCurrent(), teacupMotor.getDeviceTemp())
                .isOK();
        inputs.feederMotorRightConnected = BaseStatusSignal.refreshAll(
                        feederRight.getMotorVoltage(), feederRight.getSupplyCurrent(), feederRight.getDeviceTemp())
                .isOK();
        inputs.feederMotorRightConnected = BaseStatusSignal.refreshAll(
                        feederLeft.getMotorVoltage(), feederLeft.getSupplyCurrent(), feederLeft.getDeviceTemp())
                .isOK();

        inputs.teacupVoltage = teacupMotor.getMotorVoltage().getValueAsDouble();
        inputs.slurpVoltage = feederRight.getMotorVoltage().getValueAsDouble();
        inputs.slurpFollowerVoltage = feederLeft.getMotorVoltage().getValueAsDouble();
        inputs.teacupCurrent = teacupMotor.getSupplyCurrent().getValueAsDouble();
        inputs.slurpCurrent = feederRight.getSupplyCurrent().getValueAsDouble();
        inputs.slurpFollowerCurrent = feederLeft.getSupplyCurrent().getValueAsDouble();
        inputs.teacupTemperature = teacupMotor.getDeviceTemp().getValueAsDouble();
        inputs.slurpTemperature = feederRight.getDeviceTemp().getValueAsDouble();
        inputs.slurpFollowerTemperature = feederLeft.getDeviceTemp().getValueAsDouble();
        inputs.beamBreakTripped = !beamBreakSensor.get();
    }

    @Override
    public void setTargetVoltages(double teacupVoltage, double slurpVoltage) {
        teacupMotor.setControl(teacupRequest.withOutput(teacupVoltage));
        feederRight.setControl(slurpRequest.withOutput(slurpVoltage));
        feederLeft.setControl(slurpRequest.withOutput(-slurpVoltage));
    }
}
