package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Alert;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem.
 * The intake subsystem is responsible for controlling the intake motors. There are two motors,
 * one on the top and one on the bottom that control the intake rollers.
 * A beam break sensor is used to detect when a game piece is held so the intake doesn't pick
 * up more game pieces than it's allowed to hold.
 */
public class IntakeSubsystem extends SubsystemBase {
    private static final double COLLECTING_BOTTOM_VOLTAGE = 8.0;
    private static final double COLLECTING_TOP_VOLTAGE = 8.0;
    private static final double OUTTAKING_BOTTOM_VOLTAGE = -8.0;
    private static final double OUTTAKING_TOP_VOLTAGE = -8.0;
    private static final double REJECTING_BOTTOM_VOLTAGE = 0.0;
    private static final double REJECTING_TOP_VOLTAGE = -3.0;
    private static final double IDLING_BOTTOM_VOLTAGE = 0.0;
    private static final double IDLING_TOP_VOLTAGE = 0.0;

    public enum WantedState {
        IDLE,
        COLLECT,
        REJECT,
        EJECT,
        OFF
    }

    public enum SystemState {
        IDLING,
        COLLECTING,
        REJECTING,
        EJECTING,
        OFF
    }

    private IntakeIO io;
    private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLING;

    private BooleanSupplier beamBreakTripped;

    private final Alert intakeTopDisconnected = new Alert("Intake top motor disconnected!", Alert.AlertType.WARNING);
    private final Alert intakeBottomDisconnected =
            new Alert("Intake bottom motor disconnected!", Alert.AlertType.WARNING);

    public IntakeSubsystem(IntakeIO io, BooleanSupplier isBeamBreakTripped) {
        this.io = io;
        this.beamBreakTripped = isBeamBreakTripped;
    }

    @Override
    public void periodic() {
        // read inputs
        io.updateInputs(inputs);
        // log inputs
        Logger.processInputs("Intake", inputs);

        // process inputs
        SystemState newState = handleStateTransition();
        if (newState != systemState) {
            Logger.recordOutput("Intake/SystemState", newState.toString());
            systemState = newState;
        }

        // holds the values to apply
        double bottomMotorVoltage;
        double topMotorVoltage;

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            systemState = SystemState.IDLING;
        }

        // set voltages based on state
        switch (systemState) {
            case EJECTING:
                bottomMotorVoltage = OUTTAKING_BOTTOM_VOLTAGE;
                topMotorVoltage = OUTTAKING_TOP_VOLTAGE;
                break;
            case REJECTING:
                bottomMotorVoltage = REJECTING_BOTTOM_VOLTAGE;
                topMotorVoltage = REJECTING_TOP_VOLTAGE;
                break;
            case COLLECTING:
                bottomMotorVoltage = COLLECTING_BOTTOM_VOLTAGE;
                topMotorVoltage = COLLECTING_TOP_VOLTAGE;
                break;
            case IDLING:
                bottomMotorVoltage = IDLING_BOTTOM_VOLTAGE;
                topMotorVoltage = IDLING_TOP_VOLTAGE;
                break;
            case OFF:
            default:
                bottomMotorVoltage = 0.0;
                topMotorVoltage = 0.0;
                break;
        }

        // write outputs
        io.setBottomMotorVoltage(bottomMotorVoltage);
        io.setTopMotorVoltage(topMotorVoltage);

        Logger.recordOutput(
                "Intake/PieceEntered", inputs.bottomVelocityRPS <= 60.0 && systemState == SystemState.COLLECTING);

        // Alerts
        intakeTopDisconnected.set(!inputs.topMotorConnected);
        intakeBottomDisconnected.set(!inputs.bottomMotorConnected);
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case OFF -> SystemState.OFF;
            case EJECT -> SystemState.EJECTING;
            case REJECT -> SystemState.REJECTING;
            case COLLECT -> {
                if (beamBreakTripped.getAsBoolean()) {
                    yield SystemState.REJECTING;
                }
                yield SystemState.COLLECTING;
            }
            default -> SystemState.IDLING;
        };
    }

    public boolean pieceEntered() {
        // Measured max RPS around 64. Dipped down to 60 and below when a piece is
        // going through the rollers.
        return inputs.bottomVelocityRPS <= 60.0 && systemState == SystemState.COLLECTING;
    }

    /**
     * Sets the target state for the intake subsystem.
     * @param wantedState the target state for the intake subsystem.
     */
    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }
}
