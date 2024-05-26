package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import frc.robot.util.drivers.CanDeviceId;

public class CANCoderIOCANCoder implements CANCoderIO {
    private final CANcoder encoder;

    public CANCoderIOCANCoder(CanDeviceId canDeviceId, double offsetRotations, boolean inverted) {
        encoder = new CANcoder(canDeviceId.getDeviceNumber(), canDeviceId.getBus());

        var cancoderConfigs = new CANcoderConfiguration();
        cancoderConfigs.MagnetSensor.MagnetOffset = offsetRotations;
        cancoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        cancoderConfigs.MagnetSensor.SensorDirection =
                inverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;
        var response = encoder.getConfigurator().apply(cancoderConfigs);
        if (!response.isOK()) {
            System.out.println("CANcoder ID " + canDeviceId.getDeviceNumber() + " failed config with error "
                    + response.getDescription());
        }
    }

    @Override
    public void updateInputs(CANCoderIOInputs inputs) {

        // Update the inputs
        inputs.positionRotations = encoder.getAbsolutePosition().getValueAsDouble();
        inputs.positionDegrees = Units.rotationsToDegrees(inputs.positionRotations);
    }
}
