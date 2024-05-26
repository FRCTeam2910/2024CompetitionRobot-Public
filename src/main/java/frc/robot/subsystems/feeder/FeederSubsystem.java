package frc.robot.subsystems.feeder;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Alert;
import frc.robot.util.Angles;
import org.littletonrobotics.junction.Logger;

/**
 * Represent the Feeder subsystem.
 *
 * The feeder subsystem is responsible for controlling 3 groups of motors.
 *
 * <p>There are one motor for the <i>"teacups"</i>, which are used to control the game pieces as
 * they come in from the intake.
 *
 * <p>There is one motor for the <i>"slurp"</i> which is used to control the game pieces as they are fed
 * to the shooter.
 *
 * <p>There is one motor for the <i>"donger"</i> which is used to help pull the game pieces into the shooter or
 * eject game pieces if jammed.
 *
 * <p>A beam break sensor is used to detect when a game piece is to be held so the feeder doesn't
 * feed it directly into the shooter until ready to score.
 * The beam break sensor is also used by the Intake subsystem.
 */
public class FeederSubsystem extends SubsystemBase {
    private static final double TEACUP_VOLTAGE = 12.0;
    private static final double FEEDER_VOLTAGE_FRONTWARDS = 12.0;
    private static final double IDLE_VOLTAGE = 0.0;
    private final FeederIO io;
    private final FeederIOInputsAutoLogged ioInputs = new FeederIOInputsAutoLogged();

    public enum WantedState {
        FEED,
        HOLD,
        SCORE,
        IDLE
    }

    private enum SystemState {
        FEEDING,
        HOLDING,
        SCORING,
        IDLED
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLED;
    private Supplier<Rotation2d> currentTurretAngle;

    // The output voltages
    private double teacupSetpointVoltage = 0.0;
    private double feederSetpointVoltage = 0.0;

    private final Alert feederRightDisconnected = new Alert("Feeder Motor Right Disconnected", Alert.AlertType.WARNING);
    private final Alert feederLeftDisconnected = new Alert("Feeder Motor Left Disconnected", Alert.AlertType.WARNING);
    private final Alert teacupDisconnected = new Alert("Teacup Motor Disconnected", Alert.AlertType.WARNING);

    /**
     * Create a new FeederSubsystem.
     * @param io the IO for the feeder
     * @param currentTurretAngleSupplier the current angle of the turret
     */
    public FeederSubsystem(FeederIO io, Supplier<Rotation2d> currentTurretAngleSupplier) {
        this.io = io;
        this.currentTurretAngle = currentTurretAngleSupplier;
    }

    @Override
    public void periodic() {
        // read and log inputs
        io.updateInputs(ioInputs);
        Logger.processInputs("Feeder", ioInputs);

        // process inputs
        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            Logger.recordOutput("Feeder/SystemState", newState.toString());
            systemState = newState;
        }

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLED;
        }

        // Set the voltages based on system state and inputs
        switch (systemState) {
            case FEEDING:
                handleFeeding();
                break;
            case SCORING:
                handleScoring();
                break;
            case HOLDING:
            default:
                handleIdling();
                break;
        }

        // Write outputs
        io.setTargetVoltages(teacupSetpointVoltage, feederSetpointVoltage);

        // Log outputs
        Logger.recordOutput("Feeder/Beam break tripped?", isBeamBreakTripped());
        Logger.recordOutput("Feeder/Teacup Voltage", teacupSetpointVoltage);
        Logger.recordOutput("Feeder/Slurp Voltage", feederSetpointVoltage);

        // Alerts
        feederRightDisconnected.set(!ioInputs.feederMotorRightConnected);
        feederLeftDisconnected.set(!ioInputs.feederMotorLeftConnected);
        teacupDisconnected.set(!ioInputs.teacupMotorConnected);
    }

    /**
     * Set the target state for the feeder.
     *
     * @param wantedState the wanted state
     */
    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    /** Returns true if the beam break is tripped */
    public boolean isBeamBreakTripped() {
        return ioInputs.beamBreakTripped;
    }

    /**
     * Process the current wanted state and return the new state.
     * Handle any state transition specific logic here.
     *
     * @return the new state based on the wanted state
     */
    private SystemState handleStateTransition() {
        switch (wantedState) {
            case SCORE:
                return SystemState.SCORING;
            case HOLD:
                return SystemState.HOLDING;
            case FEED:
                if (isBeamBreakTripped()) {
                    return SystemState.HOLDING;
                }
                return SystemState.FEEDING;
            case IDLE:
            default:
                return SystemState.IDLED;
        }
    }

    private void handleScoring() {
        if (isBeamBreakTripped()) {
            teacupSetpointVoltage = IDLE_VOLTAGE;
        } else {
            if (spinTeacupsLeft()) {
                teacupSetpointVoltage = TEACUP_VOLTAGE;

            } else {
                teacupSetpointVoltage = -TEACUP_VOLTAGE;
            }
        }

        feederSetpointVoltage = FEEDER_VOLTAGE_FRONTWARDS;
    }

    private void handleIdling() {
        teacupSetpointVoltage = IDLE_VOLTAGE;
        feederSetpointVoltage = IDLE_VOLTAGE;
    }

    private void handleFeeding() {
        if (spinTeacupsLeft()) {
            teacupSetpointVoltage = TEACUP_VOLTAGE;
        } else {
            teacupSetpointVoltage = -TEACUP_VOLTAGE;
        }
        feederSetpointVoltage = FEEDER_VOLTAGE_FRONTWARDS;
    }

    private boolean spinTeacupsLeft() {
        var currentTurret = Units.radiansToDegrees(
                Angles.normalizeAnglePositive(currentTurretAngle.get().getRadians()));
        return 90 <= currentTurret && currentTurret <= 270;
    }
}
