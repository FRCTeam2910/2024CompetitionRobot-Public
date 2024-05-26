package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.Alert;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

    private static final double OFF_VOLTAGE = 0.0;
    private static final double ACCEPTABLE_RPM_ERROR = 500.0;
    private static final double LEFT_SHOT_RPM = 11000.0;
    private static final double RIGHT_SHOT_RPM = 6000.0;
    private static final double AMP_RIGHT_RPM = 2050;
    private static final double AMP_LEFT_RPM = 2050;
    private static final double OUTTAKE_RPM = 1500;
    private double leftFeedShotRPM = 5000;
    private double rightFeedShotRPM = 5000;

    private final RobotContainer container;
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public enum WantedState {
        OFF,
        SCORING,
        AMPING,
        OUTTAKE,
        FEED
    }

    public enum SystemState {
        IS_OFF,
        SCORING,
        AMPING,
        OUTTAKING,
        FEEDING
    }

    private WantedState wantedState = WantedState.SCORING;
    private SystemState systemState = SystemState.SCORING;

    private final Alert rightDisconnected;
    private final Alert leftDisconnected;

    public ShooterSubsystem(ShooterIO io, RobotContainer container) {
        this.io = io;
        this.container = container;
        rightDisconnected = new Alert("Shooter right motor disconnected!", Alert.AlertType.WARNING);
        leftDisconnected = new Alert("Shooter left motor disconnected!", Alert.AlertType.WARNING);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        SystemState newState = handleStateTransitions();
        if (newState != systemState) {
            Logger.recordOutput("Shooter/SystemState", newState.toString());
            systemState = newState;
        }

        if (DriverStation.isDisabled()) {
            systemState = SystemState.IS_OFF;
        }

        switch (systemState) {
            case IS_OFF:
                handleIsOff();
                break;
            case SCORING:
                handleScoring();
                break;
            case AMPING:
                handleAmping();
                break;
            case OUTTAKING:
                handleOuttaking();
                break;
            case FEEDING:
                handleFeeding();
                break;
        }

        // Record Outputs
        if (systemState == SystemState.IS_OFF) {
            Logger.recordOutput("Shooter/LeftTarget", OFF_VOLTAGE);
            Logger.recordOutput("Shooter/RightTarget", OFF_VOLTAGE);
        } else {
            Logger.recordOutput("Shooter/LeftTarget", LEFT_SHOT_RPM);
            Logger.recordOutput("Shooter/RightTarget", RIGHT_SHOT_RPM);
        }

        Logger.recordOutput("Shooter/WantedState", wantedState);

        // Alerts
        rightDisconnected.set(!inputs.rightConnected);
        leftDisconnected.set(!inputs.leftConnected);
    }

    private SystemState handleStateTransitions() {
        return switch (wantedState) {
            case SCORING -> SystemState.SCORING;
            case AMPING -> SystemState.AMPING;
            case FEED -> SystemState.FEEDING;
            case OUTTAKE -> SystemState.OUTTAKING;
            default -> SystemState.IS_OFF;
        };
    }

    private void handleOuttaking() {
        io.setRPM(OUTTAKE_RPM, OUTTAKE_RPM);
    }

    private void handleIsOff() {
        io.setVoltage(OFF_VOLTAGE, OFF_VOLTAGE);
    }

    private void handleScoring() {
        io.setRPM(LEFT_SHOT_RPM, RIGHT_SHOT_RPM);
    }

    private void handleAmping() {
        io.setRPM(container.getDashboard().getAmpRPM(), container.getDashboard().getAmpRPM());
    }

    private void handleFeeding() {
        io.setRPM(leftFeedShotRPM, rightFeedShotRPM);
    }

    public boolean atSpeakerSetpoint() {
        return MathUtil.isNear(LEFT_SHOT_RPM, inputs.leftRPM, ACCEPTABLE_RPM_ERROR)
                && MathUtil.isNear(RIGHT_SHOT_RPM, inputs.rightRPM, AMP_RIGHT_RPM);
    }

    public boolean atOuttakeSetpoint() {
        return MathUtil.isNear(AMP_LEFT_RPM, inputs.leftRPM, ACCEPTABLE_RPM_ERROR)
                && MathUtil.isNear(AMP_RIGHT_RPM, inputs.rightRPM, ACCEPTABLE_RPM_ERROR);
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setWantedState(WantedState wantedState, double leftFeedShotRPM, double rightFeedShotRPM) {
        this.leftFeedShotRPM = leftFeedShotRPM;
        this.rightFeedShotRPM = rightFeedShotRPM;
        this.wantedState = wantedState;
    }
}
