package frc.robot.subsystems.pivot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Alert;
import org.littletonrobotics.junction.Logger;

public class PivotSubsystem extends SubsystemBase {
    public static final double FORWARD_SOFT_LIMIT_DEGREES = 70.0;
    public static final double REVERSE_SOFT_LIMIT_DEGREES = 0.0;
    private boolean isZeroed = false;
    private boolean zeroCompleted = false;
    private double zeroTimeStamp = Double.NaN;
    private Timer zeroTimer = new Timer();
    private static final double ZERO_PERCENTAGE = -0.05;
    private static final double ZERO_VELOCITY_TIME_PERIOD = 0.25;
    private static final double PIVOT_ZERO_VELOCITY_THRESHOLD_DEGREES_PER_SECOND = 0.5;
    private static final double AMP_ZERO_VELOCITY_THRESHOLD_DEGREES_PER_SECOND = 50.0;
    private static final double AMP_BAR_AMP_PITCH = 140.0;
    private static final double PIVOT_HOMING_PITCH = -1.9;
    private static final double AMP_HOMING_PITCH = -3.0;
    private static final double ACCEPTABLE_PITCH_ERROR_DEGREES = 1.0;
    private static final double AMP_BAR_COUPLING_COEFFICIENT = (19.0 / 49.0) * (24.0 / 40.0);
    private double setpointDegrees = 0.0;
    private BooleanSupplier isClimberStowed;

    private boolean isInitialized = false;

    public enum WantedState {
        IDLE,
        HOME,
        MOVE_TO_TARGET,
        AMP,
        CLIMB
    }

    public enum SystemState {
        IS_IDLE,
        HOMING,
        MOVING,
        AMPING,
        CLIMBING
    }

    private WantedState wantedState = WantedState.HOME;
    private SystemState systemState = SystemState.HOMING;
    private PivotIO pivotIO;
    private AmpBarIO ampBarIO;
    private RobotContainer container;
    private PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();
    private AmpBarIOInputsAutoLogged ampBarInputs = new AmpBarIOInputsAutoLogged();

    private Alert pivotDisconnected = new Alert("Pivot motor disconnected!", Alert.AlertType.WARNING);
    private Alert ampBarDisconnected = new Alert("AmpBar motor disconnected!", Alert.AlertType.WARNING);

    public PivotSubsystem(
            PivotIO pivotIO, AmpBarIO ampBarIO, BooleanSupplier isClimberStowed, RobotContainer container) {
        this.pivotIO = pivotIO;
        this.ampBarIO = ampBarIO;
        this.container = container;
        this.isClimberStowed = isClimberStowed;
        zeroTimer.reset();
    }

    @Override
    public void periodic() {
        pivotIO.updateInputs(pivotInputs);
        ampBarIO.updateInputs(ampBarInputs);
        Logger.processInputs("Pivot", pivotInputs);
        Logger.processInputs("AmpBar", ampBarInputs);

        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Pivot/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.IS_IDLE;
        }

        if (DriverStation.isTeleop()) {
            if (!isClimberStowed.getAsBoolean()) {
                pivotIO.setSetpointInDegrees(0.0);
                ampBarIO.setSetpointInDegrees(0.0);
            }
        }

        switch (systemState) {
            case HOMING:
                handleHoming();
                break;
            case MOVING:
                if (isZeroed) {
                    pivotIO.setSetpointInDegrees(setpointDegrees);
                    ampBarIO.setSetpointInDegrees(-setpointDegrees * AMP_BAR_COUPLING_COEFFICIENT);
                } else {
                    wantedState = WantedState.HOME;
                    handleHoming();
                }
                break;
            case AMPING:
                if (isZeroed) {
                    pivotIO.setSetpointInDegrees(container.getDashboard().getAmpPitch());
                    ampBarIO.setSetpointInDegrees(AMP_BAR_AMP_PITCH);
                } else {
                    wantedState = WantedState.HOME;
                    handleHoming();
                }
                break;
            case CLIMBING:
                if (isZeroed) {
                    pivotIO.setPercentage(0.0);
                    ampBarIO.setSetpointInDegrees(AMP_BAR_AMP_PITCH);
                } else {
                    wantedState = WantedState.HOME;
                    handleHoming();
                }
                break;
            case IS_IDLE:
            default:
                pivotIO.setPercentage(0.0);
                ampBarIO.setPercentage(0.0);
        }

        // Write outputs
        Logger.recordOutput("Pivot/WantedState", wantedState);
        Logger.recordOutput("Pivot/PitchSetpointDegrees", setpointDegrees);
        Logger.recordOutput("Pivot/IsZeroed", isZeroed);

        // Alerts
        pivotDisconnected.set(!pivotInputs.connected);
        ampBarDisconnected.set(!ampBarInputs.connected);
    }

    private SystemState handleStateTransitions() {
        switch (wantedState) {
            case HOME:
                zeroCompleted = false;
                if (!DriverStation.isDisabled()) {
                    if (Math.abs(pivotInputs.pivotVelocityDegrees) < PIVOT_ZERO_VELOCITY_THRESHOLD_DEGREES_PER_SECOND
                            && Math.abs(ampBarInputs.ampBarVelocityDegrees)
                                    < AMP_ZERO_VELOCITY_THRESHOLD_DEGREES_PER_SECOND) {
                        if (!Double.isFinite(zeroTimeStamp)) {
                            zeroTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.HOMING;
                        } else if ((Timer.getFPGATimestamp() - zeroTimeStamp) >= ZERO_VELOCITY_TIME_PERIOD) {
                            pivotIO.setHomingPosition(PIVOT_HOMING_PITCH);
                            ampBarIO.setHomingPosition(AMP_HOMING_PITCH);
                            isZeroed = true;
                            zeroCompleted = true;
                            wantedState = WantedState.IDLE;
                            zeroTimeStamp = Double.NaN;
                            return SystemState.IS_IDLE;
                        } else {
                            return SystemState.HOMING;
                        }
                    }
                    zeroTimeStamp = Double.NaN;
                    return SystemState.HOMING;
                } else {
                    return SystemState.HOMING;
                }
            case MOVE_TO_TARGET:
                return SystemState.MOVING;
            case AMP:
                return SystemState.AMPING;
            case CLIMB:
                return SystemState.CLIMBING;
            case IDLE:
            default:
                return SystemState.IS_IDLE;
        }
    }

    private void handleHoming() {
        if (isZeroed) {
            pivotIO.disableSoftLimits();
            ampBarIO.disableSoftLimits();
            isZeroed = false;
        }
        ampBarIO.setPercentage(ZERO_PERCENTAGE);
        pivotIO.setPercentage(ZERO_PERCENTAGE);
    }

    public Command defaultCommand() {
        // implicitly require 'this'
        // return a command that initializes once and then does nothing because the periodic method handles everything
        return this.run(() -> {
            if (!isInitialized) {
                setWantedState(WantedState.IDLE);
                isInitialized = true;
            }
        });
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(WantedState wantedState, Rotation2d angle) {
        this.wantedState = wantedState;
        setTargetPitchRotation2d(angle);
    }

    public void setTargetPitchRotation2d(Rotation2d angle) {
        double clampedDegrees =
                MathUtil.clamp(angle.getDegrees(), REVERSE_SOFT_LIMIT_DEGREES, FORWARD_SOFT_LIMIT_DEGREES);
        setpointDegrees = clampedDegrees;
    }

    public boolean zeroCompleted() {
        return zeroCompleted;
    }

    public void setZeroCompleted(boolean zeroCompleted) {
        this.zeroCompleted = zeroCompleted;
    }

    public boolean pivotAtSetpoint() {
        return MathUtil.isNear(setpointDegrees, pivotInputs.pivotPositionDegrees, ACCEPTABLE_PITCH_ERROR_DEGREES);
    }

    public void setAngle(Rotation2d pivotAngle, Rotation2d ampBarAngle) {
        pivotIO.setHomingPosition(pivotAngle.getDegrees());
        ampBarIO.setHomingPosition(ampBarAngle.getDegrees());
        wantedState = WantedState.IDLE;
        systemState = SystemState.IS_IDLE;
        isZeroed = true;
        zeroCompleted = true;
        zeroTimeStamp = Double.NaN;
    }

    public double getCurrentPosition() {
        return pivotInputs.pivotPositionDegrees;
    }
}
