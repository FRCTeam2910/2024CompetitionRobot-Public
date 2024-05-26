package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.config.PortConfiguration;
import frc.robot.util.drivers.Phoenix6Util;
import frc.robot.util.drivers.TalonFXFactory;

public class ShooterIOPhoenix6 implements ShooterIO {
    private static final int SLOT = 0;

    private final TalonFX leftShooter;
    private final TalonFX rightShooter;

    private final VoltageOut leftShooterVoltageControl = new VoltageOut(0, true, false, false, false);
    private final VoltageOut rightShooterVoltageControl = new VoltageOut(0, true, false, false, false);

    private final VelocityVoltage leftShooterVelocityControl =
            new VelocityVoltage(0, 0, true, 0, SLOT, false, false, false);
    private final VelocityVoltage rightShooterVelocityControl =
            new VelocityVoltage(0, 0, true, 0, SLOT, false, false, false);

    // 2 motor rotations to 5 flywheel rotations
    private final double SHOOTER_GEAR_RATIO = 2.0 / 5.0;

    public ShooterIOPhoenix6(PortConfiguration ports) {
        leftShooter = TalonFXFactory.createDefaultTalon(ports.shooterLeftMotorID);
        rightShooter = TalonFXFactory.createDefaultTalon(ports.shooterRightMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;

        config.Slot0.kV = 0.14;
        config.Slot0.kP = 0.4;

        Phoenix6Util.applyAndCheckConfiguration(leftShooter, config);
        Phoenix6Util.applyAndCheckConfiguration(rightShooter, config);

        leftShooter.setInverted(true);
        rightShooter.setInverted(false);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.rightConnected = BaseStatusSignal.refreshAll(
                        rightShooter.getMotorVoltage(),
                        rightShooter.getSupplyCurrent(),
                        rightShooter.getDeviceTemp(),
                        rightShooter.getVelocity())
                .isOK();
        inputs.leftConnected = BaseStatusSignal.refreshAll(
                        leftShooter.getMotorVoltage(),
                        leftShooter.getSupplyCurrent(),
                        leftShooter.getDeviceTemp(),
                        leftShooter.getVelocity())
                .isOK();
        inputs.leftVoltage = leftShooter.getMotorVoltage().getValueAsDouble();
        inputs.rightVoltage = rightShooter.getMotorVoltage().getValueAsDouble();
        inputs.leftCurrent = leftShooter.getSupplyCurrent().getValueAsDouble();
        inputs.rightCurrent = rightShooter.getSupplyCurrent().getValueAsDouble();
        inputs.leftTemperature = leftShooter.getDeviceTemp().getValueAsDouble();
        inputs.rightTemperature = rightShooter.getDeviceTemp().getValueAsDouble();
        inputs.leftRPM = (leftShooter.getRotorVelocity().getValueAsDouble() * 60) / SHOOTER_GEAR_RATIO;
        inputs.rightRPM = (rightShooter.getRotorVelocity().getValueAsDouble() * 60) / SHOOTER_GEAR_RATIO;
    }

    @Override
    public void setVoltage(double leftAppliedVolts, double rightAppliedVolts) {
        leftShooter.setControl(leftShooterVoltageControl.withOutput(leftAppliedVolts));
        rightShooter.setControl(rightShooterVoltageControl.withOutput(rightAppliedVolts));
    }

    @Override
    public void setRPM(double leftTargetRPM, double rightTargetRPM) {

        double leftApplied = leftTargetRPM * SHOOTER_GEAR_RATIO / 60.0;
        double rightApplied = rightTargetRPM * SHOOTER_GEAR_RATIO / 60.0;

        leftShooter.setControl(leftShooterVelocityControl.withVelocity(leftApplied));
        rightShooter.setControl(rightShooterVelocityControl.withVelocity(rightApplied));
    }
}
