package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import frc.robot.config.PortConfiguration;
import frc.robot.util.drivers.Phoenix6Util;
import frc.robot.util.drivers.TalonFXFactory;

public class IntakeIOPhoenix6 implements IntakeIO {
    private final TalonFX topMotor;
    private final TalonFX bottomMotor;

    private final VoltageOut topRequest = new VoltageOut(0.0, true, false, false, false);
    private final VoltageOut bottomRequest = new VoltageOut(0.0, true, false, false, false);

    public IntakeIOPhoenix6(PortConfiguration ports) {
        topMotor = TalonFXFactory.createDefaultTalon(ports.intakeTopMotorID);
        bottomMotor = TalonFXFactory.createDefaultTalon(ports.intakeBottomMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 80;
        config.Voltage.PeakForwardVoltage = 12.0;
        config.Voltage.PeakReverseVoltage = -12.0;
        config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

        Phoenix6Util.applyAndCheckConfiguration(topMotor, config);
        Phoenix6Util.applyAndCheckConfiguration(bottomMotor, config);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.topMotorConnected = BaseStatusSignal.refreshAll(
                        topMotor.getMotorVoltage(),
                        topMotor.getSupplyCurrent(),
                        topMotor.getDeviceTemp(),
                        topMotor.getVelocity())
                .isOK();
        inputs.bottomMotorConnected = BaseStatusSignal.refreshAll(
                        bottomMotor.getMotorVoltage(),
                        bottomMotor.getSupplyCurrent(),
                        bottomMotor.getDeviceTemp(),
                        bottomMotor.getVelocity())
                .isOK();
        inputs.topVoltage = topMotor.getMotorVoltage().getValueAsDouble();
        inputs.topCurrent = topMotor.getSupplyCurrent().getValueAsDouble();
        inputs.topTemperature = topMotor.getDeviceTemp().getValueAsDouble();
        inputs.topVelocityRPS = topMotor.getVelocity().getValueAsDouble();
        inputs.bottomVoltage = bottomMotor.getMotorVoltage().getValueAsDouble();
        inputs.bottomCurrent = bottomMotor.getSupplyCurrent().getValueAsDouble();
        inputs.bottomTemperature = bottomMotor.getDeviceTemp().getValueAsDouble();
        inputs.bottomVelocityRPS = bottomMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setTopMotorVoltage(double voltage) {
        topMotor.setControl(topRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }

    @Override
    public void setBottomMotorVoltage(double voltage) {
        bottomMotor.setControl(bottomRequest.withOutput(MathUtil.clamp(voltage, -12.0, 12.0)));
    }
}
