package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.util.Units;
import frc.robot.config.Pigeon2Constants;
import frc.robot.util.drivers.CanDeviceId;

/**
 * The Pigeon2 is an IMU sensor that measures orientation. We can read
 * the yaw, pitch, and roll and angular velocity from it.
 */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 gyro;
    private final StatusSignal<Double> yawSignal;
    private final StatusSignal<Double> angularVelocitySignal;
    private final StatusSignal<Double> rollSignal;
    private final StatusSignal<Double> rollVelocitySignal;
    private final StatusSignal<Double> pitchSignal;
    private final StatusSignal<Double> pitchVelocitySignal;
    private final StatusSignal<Double> accelerationXSignal;
    private final StatusSignal<Double> accelerationYSignal;

    private final Pigeon2Constants constants;

    /**
     * Pigeon2 gyroIO implementation.
     *
     * @param constants The constants for the gyroscope.
     */
    public GyroIOPigeon2(Pigeon2Constants constants) {
        this.constants = constants;
        gyro = new Pigeon2(constants.CanDeviceId.getDeviceNumber(), constants.CanDeviceId.getBus());
        Pigeon2Configuration config = new Pigeon2Configuration();
        config.MountPose.MountPoseYaw = constants.MountPose;
        config.MountPose.MountPosePitch = 180;
        config.GyroTrim.GyroScalarZ = constants.GyroScalarZ;
        gyro.getConfigurator().apply(config);

        yawSignal = gyro.getYaw().clone();
        angularVelocitySignal = gyro.getAngularVelocityZDevice().clone();

        rollSignal = gyro.getRoll().clone();
        rollVelocitySignal = gyro.getAngularVelocityXWorld().clone();

        pitchSignal = gyro.getPitch().clone();
        pitchVelocitySignal = gyro.getAngularVelocityYWorld().clone();

        accelerationXSignal = gyro.getAccelerationX().clone();
        accelerationYSignal = gyro.getAccelerationY().clone();

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                yawSignal,
                angularVelocitySignal,
                rollSignal,
                rollVelocitySignal,
                pitchSignal,
                pitchVelocitySignal,
                accelerationXSignal,
                accelerationYSignal);

        Phoenix6Odometry.getInstance().registerSignal(gyro, yawSignal);
        Phoenix6Odometry.getInstance().registerSignal(gyro, angularVelocitySignal);
        Phoenix6Odometry.getInstance().registerSignal(gyro, rollSignal);
        Phoenix6Odometry.getInstance().registerSignal(gyro, rollVelocitySignal);
        Phoenix6Odometry.getInstance().registerSignal(gyro, pitchSignal);
        Phoenix6Odometry.getInstance().registerSignal(gyro, pitchVelocitySignal);
        Phoenix6Odometry.getInstance().registerSignal(gyro, accelerationXSignal);
        Phoenix6Odometry.getInstance().registerSignal(gyro, accelerationYSignal);

        gyro.optimizeBusUtilization();
    }

    @Override
    public synchronized void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(
                        yawSignal,
                        angularVelocitySignal,
                        rollSignal,
                        rollVelocitySignal,
                        pitchSignal,
                        pitchVelocitySignal,
                        accelerationXSignal,
                        accelerationYSignal)
                .isOK();

        inputs.yaw =
                Units.degreesToRadians(BaseStatusSignal.getLatencyCompensatedValue(yawSignal, angularVelocitySignal));
        inputs.pitch =
                Units.degreesToRadians(BaseStatusSignal.getLatencyCompensatedValue(pitchSignal, pitchVelocitySignal));
        inputs.angularVelocity = Units.degreesToRadians(angularVelocitySignal.getValue());
        inputs.angularVelocityDegrees = angularVelocitySignal.getValue();
        inputs.roll =
                Units.degreesToRadians(BaseStatusSignal.getLatencyCompensatedValue(rollSignal, rollVelocitySignal));
        inputs.accelerationX = accelerationXSignal.getValue();
        inputs.accelerationY = accelerationYSignal.getValue();
    }

    @Override
    public CanDeviceId getCanDeviceId() {
        return constants.CanDeviceId;
    }
}
