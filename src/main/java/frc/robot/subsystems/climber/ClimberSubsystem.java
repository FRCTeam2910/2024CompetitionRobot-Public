package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.util.Alert;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
    private boolean zeroCompleted = false;
    private double zeroTimeStamp = Double.NaN;
    private Timer zeroTimer = new Timer();
    private static final double ZERO_PERCENTAGE = -0.08;
    private static final double ZERO_VELOCITY_TIME_PERIOD = 0.25;
    private static final double CLIMBER_ZERO_VELOCITY_THRESHOLD_MOTOR_ROTATIONS_PER_SECOND = 0.5 * 0.05;
    private static final double CLIMBER_HOMING_PITCH = -8.152429;
    private double setPointInMotorRotations = 0.0;
    private double setPointPercentage = 0.0;

    public enum WantedState {
        IDLE,
        HOME,
        MOVE_TO_TARGET,
        PERCENT_OUTPUT,
    }

    public enum SystemState {
        IS_IDLE,
        HOMING,
        MOVING,
        PERCENT_OUTPUT,
    }

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IS_IDLE;
    private ClimberIO climberIO;
    private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

    private Alert climberDisconnected = new Alert("Climber motor disconnected!", Alert.AlertType.WARNING);

    public ClimberSubsystem(ClimberIO climberIO) {
        this.climberIO = climberIO;
        zeroTimer.reset();
    }

    @Override
    public void periodic() {
        climberIO.updateInputs(climberInputs);
        Logger.processInputs("Climber", climberInputs);

        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Climber/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.IS_IDLE;
        }

        switch (systemState) {
            case HOMING:
                handleHoming();
                break;
            case MOVING:
                if (zeroCompleted) {
                    climberIO.setSetpoint(setPointInMotorRotations);
                } else {
                    wantedState = WantedState.HOME;
                    handleHoming();
                }
                break;
            case PERCENT_OUTPUT:
                if (zeroCompleted) {
                    climberIO.setPercentage(setPointPercentage);
                } else {
                    wantedState = WantedState.HOME;
                    handleHoming();
                }
                break;
            case IS_IDLE:
            default:
                climberIO.setPercentage(0.0);
        }

        // Write outputs
        Logger.recordOutput("Climber/WantedState", wantedState);
        Logger.recordOutput("Climber/PitchSetpointMotorRotations", setPointInMotorRotations);
        Logger.recordOutput("Climber/IsZeroed", zeroCompleted);

        // Alerts
        climberDisconnected.set(!climberInputs.connected);
    }

    private SystemState handleStateTransitions() {
        switch (wantedState) {
            case HOME:
                if (!DriverStation.isDisabled()) {
                    if (Math.abs(climberInputs.climberVelocityMotorRotations)
                            < CLIMBER_ZERO_VELOCITY_THRESHOLD_MOTOR_ROTATIONS_PER_SECOND) {
                        if (!Double.isFinite(zeroTimeStamp)) {
                            zeroTimeStamp = Timer.getFPGATimestamp();
                            return SystemState.HOMING;
                        } else if ((Timer.getFPGATimestamp() - zeroTimeStamp) >= ZERO_VELOCITY_TIME_PERIOD) {
                            climberIO.setHomingPosition(CLIMBER_HOMING_PITCH);
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
                if (MathUtil.isNear(
                        0.0, RobotState.getInstance().getCurrentTurretAngle().getDegrees(), 5.0)) {
                    return SystemState.MOVING;
                } else {
                    return SystemState.IS_IDLE;
                }
            case PERCENT_OUTPUT:
                if (MathUtil.isNear(
                        0.0, RobotState.getInstance().getCurrentTurretAngle().getDegrees(), 5.0)) {
                    return SystemState.PERCENT_OUTPUT;
                } else {
                    return SystemState.IS_IDLE;
                }
            case IDLE:
            default:
                return SystemState.IS_IDLE;
        }
    }

    private void handleHoming() {
        if (zeroCompleted) {
            climberIO.disableSoftLimits();
            zeroCompleted = false;
        }
        climberIO.setPercentage(ZERO_PERCENTAGE);
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(WantedState wantedState, double motorRotations) {
        this.wantedState = wantedState;
        setTarget(motorRotations);
    }

    public void setWantedStatePercentOutput(WantedState wantedState, double setPointPercentage) {
        this.wantedState = wantedState;
        this.setPointPercentage = setPointPercentage;
    }

    public void setTarget(double setPointInMotorRotations) {
        this.setPointInMotorRotations = setPointInMotorRotations;
    }

    public boolean zeroCompleted() {
        return zeroCompleted;
    }

    public void setZeroCompleted(boolean zeroCompleted) {
        this.zeroCompleted = zeroCompleted;
    }

    public void setAngle(Rotation2d angle) {
        zeroCompleted = true;
        zeroTimeStamp = Double.NaN;
        wantedState = ClimberSubsystem.WantedState.IDLE;
        systemState = ClimberSubsystem.SystemState.IS_IDLE;
        climberIO.setHomingPosition(angle.getDegrees());
    }

    public boolean isStowed() {
        return climberInputs.climberPositionMotorRotations < 2.0 && zeroCompleted;
    }
}
