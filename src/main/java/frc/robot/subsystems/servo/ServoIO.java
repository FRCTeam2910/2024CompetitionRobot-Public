package frc.robot.subsystems.servo;

import java.util.function.UnaryOperator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.subsystems.servo.ServoSubsystem.ControlState;
import org.littletonrobotics.junction.AutoLog;

public interface ServoIO {

    /**
     * Reads sensor data (hardware or simulation) and updates the inputs parameter.
     *
     * @param inputs Contains the defaults for the input values listed above.
     */
    default void updateInputs(ServoIOInputs inputs) {}

    @AutoLog
    class ServoIOInputs {
        public boolean connected = true;
        public double timestamp = 0.0;
        public double positionRotations = 0.0; // motor rotations
        public double positionUnits = 0.0;
        public double velocityRotationsSecond = 0.0;
        public double previousVelocityRotationsSecond = 0.0;
        public double outputPercent = 0.0;
        public double outputVoltage = 0.0;
        public double masterCurrent = 0.0;
        public double errorRotations = 0.0;
        public boolean resetOccured = false;
        public double activeTrajectoryPosition = 0.0;
        public double activeTrajectoryVelocity = 0.0;
        public double activeTrajectoryAcceleration = 0.0;
    }

    /**
     * Represents the desired outputs for the servo mechanism.
     */
    public class ServoIOOutputs {
        // OUTPUTS
        public double demand; // position (motor rots) or percent output
        public double feedforward;
    }

    default BaseStatusSignal[] getSignals() {
        return new BaseStatusSignal[0];
    }

    default void handleMasterReset(boolean reset) {}

    default void zeroSensors() {}

    default void setSensorPosition(double units) {}

    default void changeTalonConfig(UnaryOperator<TalonFXConfiguration> configChanger) {}

    default void writeConfigs() {}

    default void handleReset() {}

    default void setSupplyCurrentLimit(double value, boolean enable, boolean unchecked) {}

    default void setStatorCurrentLimit(double value, boolean enable, boolean unchecked) {}

    default void setMotionMagicConfigs(double accel, double jerk, boolean unchecked) {}

    default void writeOutputs(ControlState controlState, double demand, double feedforward) {}

    default void stop() {}

    default TalonFXConfiguration getConfig() {
        return new TalonFXConfiguration();
    }
}
