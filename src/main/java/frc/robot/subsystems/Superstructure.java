package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.config.FieldConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
    private FeederSubsystem feeder;
    private PivotSubsystem pivot;
    private ShooterSubsystem shooter;
    private SwerveSubsystem swerve;
    private TurretSubsystem turret;
    private IntakeSubsystem intake;
    private ClimberSubsystem climber;
    private RobotContainer container;
    private final Timer climberHeadingLockTimer = new Timer();

    private boolean subwooferShotMode = false;
    private boolean feedShotMode = false;

    public enum WantedSuperState {
        PREPARING_AMP_SHOT,
        SHOOT_AMP,
        SHOOT_SPEAKER,
        FORCE_FEED,
        STOPPED,
        HOME_ALL_PIVOTS,
        INTAKE_PIECE,
        FORCE_SPEAKER_SHOT,
        REGULAR_STATE,
        TRACK_WITHOUT_PIVOT,
        OUTTAKE_THROUGH_COLLECTOR,
        AUTO_STATIONARY_SHOT,
        DEPLOY_CLIMBER,
        RETRACT_CLIMBER,
        MOVE_CLIMBER_TO_STOW,
        HOME_ONLY_PIVOT,
        MOVE_PIVOT_TO_ZERO,
        PREPARE_MANUAL_PIVOT_AND_TURRET,
        SHOOT_MANUAL_PIVOT_AND_TURRET,
        PREPARE_SUBWOOFER_SHOT,
        SHOOT_SUBWOOFER_SHOT,
        TRACK_FEED_SHOT,
        SHOOT_FEED_SHOT,
        INTAKE_IN_FEED_MODE,
        PREPARE_SPEAKER_SHOT,
    }

    public enum CurrentSuperState {
        PREPARING_AMP_SHOT,
        SHOOTING_AMP,
        PREPARING_SPEAKER_SHOT,
        FORCE_FEED,
        READY_FOR_SPEAKER_SHOT,
        STOPPED,
        HOME_ALL_PIVOTS,
        OUTTAKE_PIECE,
        INTAKE_PIECE,
        HOLD_PIECE,
        FORCE_SPEAKER_SHOT,
        NO_PIECE,
        TRACK_WITHOUT_PIVOT,
        SHOOTER_OUTTAKE,
        OUTTAKE_THROUGH_COLLECTOR,
        AUTO_STATIONARY_SHOT,
        DEPLOY_CLIMBER,
        RETRACT_CLIMBER,
        MOVE_CLIMBER_TO_STOW,
        HOME_ONLY_PIVOT,
        MOVE_PIVOT_TO_ZERO,
        PREPARE_MANUAL_PIVOT_AND_TURRET,
        SHOOT_MANUAL_PIVOT_AND_TURRET,
        PREPARE_SUBWOOFER_SHOT,
        SHOOT_SUBWOOFER_SHOT,
        PREPARE_FEED_SHOT,
        SHOOT_FEED_SHOT,
        INTAKE_IN_FEED_MODE,
        PREPARE_SPEAKER_SHOT,
    }

    private WantedSuperState wantedSuperState = WantedSuperState.STOPPED;
    private CurrentSuperState currentSuperState = CurrentSuperState.STOPPED;
    private CurrentSuperState previousSuperState;
    RobotState.AimingParameters aimingParameters =
            new RobotState.AimingParameters(new Rotation2d(), new Rotation2d(), new Translation2d(), 0.0);

    private static final double CLIMBER_MOTOR_ROTATIONS_CLIMB = 105.0;
    private static final double CLIMBER_MOTOR_ROTATIONS_CLIMB_SECONDARY = 45.0;
    private Rotation2d manualTurretSetpoint = new Rotation2d();
    private Rotation2d manualPitchSetpoint = new Rotation2d();

    public Superstructure(
            FeederSubsystem feeder,
            PivotSubsystem pivot,
            ShooterSubsystem shooter,
            SwerveSubsystem swerve,
            TurretSubsystem turret,
            IntakeSubsystem intake,
            ClimberSubsystem climber,
            RobotContainer container) {
        this.feeder = feeder;
        this.pivot = pivot;
        this.shooter = shooter;
        this.swerve = swerve;
        this.turret = turret;
        this.intake = intake;
        this.climber = climber;
        this.container = container;

        climberHeadingLockTimer.reset();
    }

    @Override
    public void periodic() {
        double percentageOfThreeMetersPerSecond = Math.hypot(
                        RobotState.getInstance().getChassisSpeeds().vxMetersPerSecond,
                        RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond)
                / 3.0;
        aimingParameters = RobotState.getInstance()
                .getAimingParameters(0.6 * percentageOfThreeMetersPerSecond, 0.25 * percentageOfThreeMetersPerSecond);
        currentSuperState = handleStateTransitions();
        applyStates();

        Logger.recordOutput("TeleopShotReady/PivotAtSetpoint", pivot.pivotAtSetpoint());
        Logger.recordOutput("TeleopShotReady/PivotGreaterThan10", pivot.getCurrentPosition() > 10.0);
        Logger.recordOutput("TeleopShotReady/ShooterAtSpeakerSetpoint", shooter.atSpeakerSetpoint());
        Logger.recordOutput(
                "TeleopShotReady/AccelerationVectorUnder12",
                RobotState.getInstance().getLastAccelerationVector() < 0.12);
        Logger.recordOutput(
                "TeleopShotReady/HasTarget", RobotState.getInstance().hasTarget());
        Logger.recordOutput(
                "TeleopShotReady/CameraWithin8Meters", RobotState.getInstance().getVisionHorizontalDistance() <= 8.0);
        Logger.recordOutput(
                "TeleopShotReady/PredictedPoseWithin8Meters",
                aimingParameters.effectiveDistance().getX() <= 8.0);

        Logger.recordOutput("DesiredSuperstate", wantedSuperState);
        if (currentSuperState != previousSuperState) {
            Logger.recordOutput("CurrentSuperstate", currentSuperState);
        }

        Logger.recordOutput(
                "AimingParameters/AdjustedTurretAngleDegrees",
                aimingParameters.turretAimingAngle().getDegrees());
        Logger.recordOutput("AimingParameters/EffectiveDistance", aimingParameters.effectiveDistance());

        Logger.recordOutput("FeedShotDistance", RobotState.getInstance().getDistanceToFeedTarget());
    }

    private CurrentSuperState handleStateTransitions() {
        previousSuperState = currentSuperState;
        switch (wantedSuperState) {
            case REGULAR_STATE:
                if (subwooferShotMode) {
                    currentSuperState = CurrentSuperState.PREPARE_SUBWOOFER_SHOT;
                } else if (feedShotMode) {
                    currentSuperState = CurrentSuperState.PREPARE_FEED_SHOT;
                } else {
                    currentSuperState = hasPiece() ? CurrentSuperState.HOLD_PIECE : CurrentSuperState.NO_PIECE;
                }
                break;
            case INTAKE_PIECE:
                currentSuperState = CurrentSuperState.INTAKE_PIECE;
                break;
            case PREPARING_AMP_SHOT:
                currentSuperState = CurrentSuperState.PREPARING_AMP_SHOT;
                break;
            case SHOOT_AMP:
                currentSuperState = CurrentSuperState.SHOOTING_AMP;
                break;
            case SHOOT_SPEAKER:
                currentSuperState = areSystemsReadyForTeleopShot()
                        ? CurrentSuperState.READY_FOR_SPEAKER_SHOT
                        : CurrentSuperState.PREPARING_SPEAKER_SHOT;
                break;
            case FORCE_FEED:
                currentSuperState = CurrentSuperState.FORCE_FEED;
                break;
            case HOME_ALL_PIVOTS:
                currentSuperState = CurrentSuperState.HOME_ALL_PIVOTS;
                break;
            case FORCE_SPEAKER_SHOT:
                currentSuperState = CurrentSuperState.FORCE_SPEAKER_SHOT;
                break;
            case TRACK_WITHOUT_PIVOT:
                currentSuperState = CurrentSuperState.TRACK_WITHOUT_PIVOT;
                break;
            case OUTTAKE_THROUGH_COLLECTOR:
                currentSuperState = CurrentSuperState.OUTTAKE_THROUGH_COLLECTOR;
                break;
            case AUTO_STATIONARY_SHOT:
                currentSuperState = areSystemsReadyForAutoShot()
                        ? CurrentSuperState.READY_FOR_SPEAKER_SHOT
                        : CurrentSuperState.PREPARING_SPEAKER_SHOT;
                break;
            case DEPLOY_CLIMBER:
                currentSuperState = CurrentSuperState.DEPLOY_CLIMBER;
                break;
            case RETRACT_CLIMBER:
                currentSuperState = CurrentSuperState.RETRACT_CLIMBER;
                break;
            case MOVE_CLIMBER_TO_STOW:
                currentSuperState = CurrentSuperState.MOVE_CLIMBER_TO_STOW;
                break;
            case HOME_ONLY_PIVOT:
                currentSuperState = CurrentSuperState.HOME_ONLY_PIVOT;
                break;
            case MOVE_PIVOT_TO_ZERO:
                currentSuperState = CurrentSuperState.MOVE_PIVOT_TO_ZERO;
                break;
            case PREPARE_MANUAL_PIVOT_AND_TURRET:
                currentSuperState = CurrentSuperState.PREPARE_MANUAL_PIVOT_AND_TURRET;
                break;
            case SHOOT_MANUAL_PIVOT_AND_TURRET:
                currentSuperState = CurrentSuperState.SHOOT_MANUAL_PIVOT_AND_TURRET;
                break;
            case PREPARE_SUBWOOFER_SHOT:
                currentSuperState = CurrentSuperState.PREPARE_SUBWOOFER_SHOT;
                break;
            case SHOOT_SUBWOOFER_SHOT:
                currentSuperState = CurrentSuperState.SHOOT_SUBWOOFER_SHOT;
                break;
            case TRACK_FEED_SHOT:
                currentSuperState = CurrentSuperState.PREPARE_FEED_SHOT;
                break;
            case SHOOT_FEED_SHOT:
                currentSuperState = CurrentSuperState.SHOOT_FEED_SHOT;
                break;
            case INTAKE_IN_FEED_MODE:
                currentSuperState = CurrentSuperState.INTAKE_IN_FEED_MODE;
                break;
            case PREPARE_SPEAKER_SHOT:
                currentSuperState = CurrentSuperState.PREPARING_SPEAKER_SHOT;
                break;
            case STOPPED:
            default:
                currentSuperState = CurrentSuperState.STOPPED;
                break;
        }
        return currentSuperState;
    }

    private void applyStates() {
        switch (currentSuperState) {
            case PREPARING_AMP_SHOT:
                prepareForAmpShot();
                break;
            case SHOOTING_AMP:
                shootAmp();
                break;
            case PREPARING_SPEAKER_SHOT:
                prepareForSpeakerShot();
                break;
            case READY_FOR_SPEAKER_SHOT:
                scoreSpeaker();
                break;
            case FORCE_FEED:
                forceFeed();
                break;
            case HOME_ALL_PIVOTS:
                homeAllPivots();
                break;
            case HOLD_PIECE:
                holdPiece();
                break;
            case NO_PIECE:
                noPiece();
                break;
            case INTAKE_PIECE:
                intakePiece();
                break;
            case FORCE_SPEAKER_SHOT:
                handleAutoForceShot();
                break;
            case TRACK_WITHOUT_PIVOT:
                handleTrackWithoutPivot();
                break;
            case SHOOTER_OUTTAKE:
                handleOuttake();
                break;
            case OUTTAKE_THROUGH_COLLECTOR:
                handleIntakeOuttake();
                break;
            case DEPLOY_CLIMBER:
                handleDeployClimber();
                break;
            case RETRACT_CLIMBER:
                handleRetractClimber();
                break;
            case HOME_ONLY_PIVOT:
                handleHomeOnlyPivot();
                break;
            case MOVE_CLIMBER_TO_STOW:
                handleMoveClimberToStow();
                break;
            case MOVE_PIVOT_TO_ZERO:
                handleMovePivotTo0();
                break;
            case PREPARE_MANUAL_PIVOT_AND_TURRET:
                handlePrepareManualTurretAndPivot();
                break;
            case SHOOT_MANUAL_PIVOT_AND_TURRET:
                handleShootManualTurretAndPivot();
                break;
            case PREPARE_SUBWOOFER_SHOT:
                handlePrepareSubwooferShot();
                break;
            case SHOOT_SUBWOOFER_SHOT:
                handleShootSubwooferShot();
                break;
            case PREPARE_FEED_SHOT:
                handleTrackFeedShot();
                break;
            case SHOOT_FEED_SHOT:
                handleShootFeedShot();
                break;
            case PREPARE_SPEAKER_SHOT:
                handlePrepareSpeakerShot();
                break;
            case INTAKE_IN_FEED_MODE:
                handleIntakeWhileInFeedingMode();
                break;
            case STOPPED:
            default:
                handleStopped();
                break;
        }
    }

    /** Checks */
    private boolean areSystemsReadyForAutoShot() {
        boolean isReady = turret.atSetpoint()
                && pivot.pivotAtSetpoint()
                && pivot.getCurrentPosition() > 10.0
                && shooter.atSpeakerSetpoint()
                && swerve.isChassisStopped()
                && RobotState.getInstance().hasTarget()
                && RobotState.getInstance().getVisionHorizontalDistance() <= 10.0;
        return isReady;
    }

    private boolean areSystemsReadyForTeleopShot() {
        boolean isReady = pivot.pivotAtSetpoint()
                && pivot.getCurrentPosition() > 10.0
                && shooter.atSpeakerSetpoint()
                && RobotState.getInstance().getLastAccelerationVector() < 0.12
                && RobotState.getInstance().getVisionHorizontalDistance() <= 8.0
                && aimingParameters.effectiveDistance().getX() <= 8.0;
        if (container.getDashboard().getRequireVisionForShot()) {
            return isReady && RobotState.getInstance().hasTarget();
        }
        return isReady;
    }

    /** Basic states - default and intake */
    private void holdPiece() {
        swerve.disableSlowdownCoefficients();
        turret.setShouldVelocityCompensationBeApplied(true);
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        if (pivot.zeroCompleted()) {
            pivot.setWantedState(
                    PivotSubsystem.WantedState.MOVE_TO_TARGET,
                    container
                            .getInterpolation()
                            .getPitch(RobotState.getInstance().getVisionHorizontalDistance()));
        }
        trackSpeakerWithTurret();
        feeder.setWantedState(FeederSubsystem.WantedState.HOLD);
        intake.setWantedState(IntakeSubsystem.WantedState.REJECT);
        climber.setWantedState(ClimberSubsystem.WantedState.MOVE_TO_TARGET, 0.0);
        if (climber.zeroCompleted()) {
            climber.setWantedState(ClimberSubsystem.WantedState.MOVE_TO_TARGET, 0.0);
        }
    }

    private void noPiece() {
        swerve.disableSlowdownCoefficients();
        turret.setShouldVelocityCompensationBeApplied(true);
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        pivot.setWantedState(
                PivotSubsystem.WantedState.MOVE_TO_TARGET,
                container.getInterpolation().getPitch(RobotState.getInstance().getVisionHorizontalDistance()));
        trackSpeakerWithTurret();
        feeder.setWantedState(FeederSubsystem.WantedState.HOLD);
        intake.setWantedState(IntakeSubsystem.WantedState.REJECT);
        if (climber.zeroCompleted()) {
            climber.setWantedState(ClimberSubsystem.WantedState.MOVE_TO_TARGET, 0.0);
        }
    }

    private void intakePiece() {
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        pivot.setWantedState(
                PivotSubsystem.WantedState.MOVE_TO_TARGET,
                container
                        .getInterpolation()
                        .getPitch(aimingParameters.effectiveDistance().getNorm()));
        trackSpeakerWithTurret();
        feeder.setWantedState(FeederSubsystem.WantedState.FEED);
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
    }

    /** Main scoring states - speaker and amp */
    private void prepareForSpeakerShot() {
        if (hasPiece()) {
            swerve.setSlowdownCoefficient(0.5, 1.0);
        }
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);

        Logger.recordOutput(
                "WithinRegularTrackingSpeeds",
                RobotState.getInstance().withinChassisSpeeds(new ChassisSpeeds(0.75, 0.75, 4 * Math.PI)));
        if (RobotState.getInstance().withinChassisSpeeds(new ChassisSpeeds(0.75, 0.75, 4 * Math.PI))) {
            trackSpeakerWithTurret();
            pivot.setWantedState(
                    PivotSubsystem.WantedState.MOVE_TO_TARGET,
                    container
                            .getInterpolation()
                            .getPitch(RobotState.getInstance().getVisionHorizontalDistance()));
        } else {
            turret.setWantedState(TurretSubsystem.WantedState.TARGET, aimingParameters.turretAimingAngle());
            pivot.setWantedState(
                    PivotSubsystem.WantedState.MOVE_TO_TARGET,
                    container
                            .getInterpolation()
                            .getPitch(aimingParameters.effectiveDistance().getNorm()));
        }

        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        feeder.setWantedState(FeederSubsystem.WantedState.FEED);
    }

    private void scoreSpeaker() {
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
        feeder.setWantedState(FeederSubsystem.WantedState.SCORE);
        if (RobotState.getInstance().withinChassisSpeeds(new ChassisSpeeds(0.75, 0.75, 4 * Math.PI))) {
            trackSpeakerWithTurret();
            pivot.setWantedState(
                    PivotSubsystem.WantedState.MOVE_TO_TARGET,
                    container
                            .getInterpolation()
                            .getPitch(RobotState.getInstance().getVisionHorizontalDistance()));
        } else {
            turret.setWantedState(TurretSubsystem.WantedState.TARGET, aimingParameters.turretAimingAngle());
            pivot.setWantedState(
                    PivotSubsystem.WantedState.MOVE_TO_TARGET,
                    container
                            .getInterpolation()
                            .getPitch(aimingParameters.effectiveDistance().getNorm()));
        }
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
    }

    private void shootAmp() {
        swerve.setSlowdownCoefficient(0.5, 1.0);
        swerve.setRotationLock(Rotation2d.fromDegrees(-90.0));
        pivot.setWantedState(PivotSubsystem.WantedState.AMP);
        shooter.setWantedState(ShooterSubsystem.WantedState.AMPING);
        turret.setWantedState(TurretSubsystem.WantedState.TARGET, Rotation2d.fromDegrees(180.0));
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
        feeder.setWantedState(FeederSubsystem.WantedState.SCORE);
    }

    private void prepareForAmpShot() {
        swerve.setSlowdownCoefficient(0.5, 1.0);
        swerve.setRotationLock(Rotation2d.fromDegrees(-90.0));
        pivot.setWantedState(PivotSubsystem.WantedState.AMP);
        shooter.setWantedState(ShooterSubsystem.WantedState.AMPING);
        turret.setWantedState(TurretSubsystem.WantedState.TARGET, Rotation2d.fromDegrees(180.0));
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
        feeder.setWantedState(FeederSubsystem.WantedState.FEED);
    }

    /** Climb states */
    private void handleMoveClimberToStow() {
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        pivot.setWantedState(PivotSubsystem.WantedState.MOVE_TO_TARGET, new Rotation2d());
        feeder.setWantedState(FeederSubsystem.WantedState.FEED);
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
        // turret.setWantedState(TurretSubsystem.WantedState.TARGET, new Rotation2d());
        turret.setWantedStateWithAbsoluteAngle(TurretSubsystem.WantedState.TARGET, new Rotation2d());
        if (turret.atSetpoint()) {
            climber.setWantedState(ClimberSubsystem.WantedState.MOVE_TO_TARGET, 0.0);
        }
        if (climber.isStowed()) {
            wantedSuperState = WantedSuperState.REGULAR_STATE;
        }
    }

    private void handleDeployClimber() {
        swerve.setSlowdownCoefficient(0.5, 1.0);
        climberHeadingLockTimer.start();
        shooter.setWantedState(ShooterSubsystem.WantedState.OFF);
        pivot.setWantedState(PivotSubsystem.WantedState.IDLE);
        feeder.setWantedState(FeederSubsystem.WantedState.IDLE);
        intake.setWantedState(IntakeSubsystem.WantedState.IDLE);
        // turret.setWantedState(TurretSubsystem.WantedState.TARGET, new Rotation2d());
        turret.setWantedStateWithAbsoluteAngle(TurretSubsystem.WantedState.TARGET, new Rotation2d());
        if (turret.atSetpoint()) {
            climber.setWantedState(ClimberSubsystem.WantedState.MOVE_TO_TARGET, CLIMBER_MOTOR_ROTATIONS_CLIMB);
            pivot.setWantedState(PivotSubsystem.WantedState.CLIMB);
        }

        // Logger.recordOutput("ClimberRotationPose", new Pose2d(8, 8, getClimbRotation()));

        if (climberHeadingLockTimer.hasElapsed(1.0)) {
            swerve.disableRotationLock();
        } else {
            swerve.setRotationLock(getClimbRotation());
        }
    }

    private void handleRetractClimber() {
        swerve.disableSlowdownCoefficients();
        shooter.setWantedState(ShooterSubsystem.WantedState.OFF);
        pivot.setWantedState(PivotSubsystem.WantedState.IDLE);
        feeder.setWantedState(FeederSubsystem.WantedState.IDLE);
        intake.setWantedState(IntakeSubsystem.WantedState.IDLE);
        // turret.setWantedState(TurretSubsystem.WantedState.TARGET, new Rotation2d());
        turret.setWantedStateWithAbsoluteAngle(TurretSubsystem.WantedState.TARGET, new Rotation2d());
        // climber.setWantedState(ClimberSubsystem.WantedState.MOVE_TO_TARGET, CLIMBER_MOTOR_ROTATIONS_CLIMB_SECONDARY);
        if (turret.atSetpoint()) {
            climber.setWantedState(
                    ClimberSubsystem.WantedState.MOVE_TO_TARGET, CLIMBER_MOTOR_ROTATIONS_CLIMB_SECONDARY);
            pivot.setWantedState(PivotSubsystem.WantedState.CLIMB);
            //            climber.setWantedStatePercentOutput(ClimberSubsystem.WantedState.PERCENT_OUTPUT, -1.0);
            //            climber.setTarget(CLIMBER_MOTOR_ROTATIONS_CLIMB_SECONDARY);
        }
    }

    /** Feed states */
    private void handleShootFeedShot() {
        swerve.setSlowdownCoefficient(0.3, 1.0);
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
        if (RobotState.getInstance().withinChassisSpeeds(new ChassisSpeeds(1.0, 1.0, 4 * Math.PI))) {
            feeder.setWantedState(FeederSubsystem.WantedState.SCORE);
        } else {
            feeder.setWantedState(FeederSubsystem.WantedState.FEED);
        }
        if (RobotState.getInstance().withinChassisSpeeds(new ChassisSpeeds(0.2, 0.2, 4 * Math.PI))) {
            var trackpose = new Pose2d();
            if (container.getDashboard().getShouldFeedFromSource()) {
                trackpose = new Pose2d(
                        FieldConstants.getFeedShotFromSource().getX(),
                        FieldConstants.getFeedShotFromSource().getY()
                                + container.getDashboard().getFeedingYOffset(),
                        new Rotation2d());
            } else {
                trackpose = new Pose2d(
                        FieldConstants.getStationaryFeedShotTarget().getX(),
                        FieldConstants.getStationaryFeedShotTarget().getY()
                                + container.getDashboard().getFeedingYOffset(),
                        new Rotation2d());
            }
            trackFeedPoseWithTurret(trackpose);
            Logger.recordOutput("FeedShotTarget", trackpose);
            Logger.recordOutput("FeedShotMode", "Stationary");
        } else {
            var trackpose = new Pose2d();
            if (container.getDashboard().getShouldFeedFromSource()) {
                trackpose = new Pose2d(
                        FieldConstants.getFeedShotFromSource().getX(),
                        FieldConstants.getFeedShotFromSource().getY()
                                + container.getDashboard().getFeedingYOffset(),
                        new Rotation2d());
            } else {
                trackpose = new Pose2d(
                        FieldConstants.getRegularFeedShotTarget().getX(),
                        FieldConstants.getRegularFeedShotTarget().getY()
                                + container.getDashboard().getFeedingYOffset(),
                        new Rotation2d());
            }
            trackFeedPoseWithTurret(trackpose);
            Logger.recordOutput("FeedShotTarget", trackpose);
            Logger.recordOutput("FeedShotMode", "Regular");
        }
        if (container.getDashboard().getShouldFeedFromSource()) {
            shooter.setWantedState(
                    ShooterSubsystem.WantedState.FEED,
                    container
                            .getInterpolation()
                            .getFeedShotRPM(RobotState.getInstance().getDistancToFeedFromSource()),
                    container
                                    .getInterpolation()
                                    .getFeedShotRPM(RobotState.getInstance().getDistancToFeedFromSource())
                            * (6.0 / 11.0));
            pivot.setWantedState(
                    PivotSubsystem.WantedState.MOVE_TO_TARGET,
                    container
                            .getInterpolation()
                            .getFeedShotPitch(RobotState.getInstance().getDistancToFeedFromSource()));
        } else {
            shooter.setWantedState(
                    ShooterSubsystem.WantedState.FEED,
                    container
                            .getInterpolation()
                            .getFeedShotRPM(RobotState.getInstance().getDistanceToFeedTarget()),
                    container
                                    .getInterpolation()
                                    .getFeedShotRPM(RobotState.getInstance().getDistanceToFeedTarget())
                            * (6.0 / 11.0));
            pivot.setWantedState(
                    PivotSubsystem.WantedState.MOVE_TO_TARGET,
                    container
                            .getInterpolation()
                            .getFeedShotPitch(RobotState.getInstance().getDistanceToFeedTarget()));
        }
    }

    private void handleTrackFeedShot() {
        swerve.disableSlowdownCoefficients();
        intake.setWantedState(IntakeSubsystem.WantedState.REJECT);
        feeder.setWantedState(FeederSubsystem.WantedState.FEED);
        if (RobotState.getInstance().withinChassisSpeeds(new ChassisSpeeds(0.2, 0.2, 4 * Math.PI))) {
            var trackpose = new Pose2d();
            if (container.getDashboard().getShouldFeedFromSource()) {
                trackpose = new Pose2d(
                        FieldConstants.getFeedShotFromSource().getX(),
                        FieldConstants.getFeedShotFromSource().getY()
                                + container.getDashboard().getFeedingYOffset(),
                        new Rotation2d());
            } else {
                trackpose = new Pose2d(
                        FieldConstants.getStationaryFeedShotTarget().getX(),
                        FieldConstants.getStationaryFeedShotTarget().getY()
                                + container.getDashboard().getFeedingYOffset(),
                        new Rotation2d());
            }
            trackFeedPoseWithTurret(trackpose);
            Logger.recordOutput("FeedShotTarget", trackpose);
            Logger.recordOutput("FeedShotMode", "Stationary");
        } else {
            var trackpose = new Pose2d();
            if (container.getDashboard().getShouldFeedFromSource()) {
                trackpose = new Pose2d(
                        FieldConstants.getFeedShotFromSource().getX(),
                        FieldConstants.getFeedShotFromSource().getY()
                                + container.getDashboard().getFeedingYOffset(),
                        new Rotation2d());
            } else {
                trackpose = new Pose2d(
                        FieldConstants.getRegularFeedShotTarget().getX(),
                        FieldConstants.getRegularFeedShotTarget().getY()
                                + container.getDashboard().getFeedingYOffset(),
                        new Rotation2d());
            }
            trackFeedPoseWithTurret(trackpose);
            Logger.recordOutput("FeedShotTarget", trackpose);
            Logger.recordOutput("FeedShotMode", "Regular");
        }
        if (container.getDashboard().getShouldFeedFromSource()) {
            shooter.setWantedState(
                    ShooterSubsystem.WantedState.FEED,
                    container
                            .getInterpolation()
                            .getFeedShotRPM(RobotState.getInstance().getDistancToFeedFromSource()),
                    container
                                    .getInterpolation()
                                    .getFeedShotRPM(RobotState.getInstance().getDistancToFeedFromSource())
                            * (6.0 / 11.0));
            pivot.setWantedState(
                    PivotSubsystem.WantedState.MOVE_TO_TARGET,
                    container
                            .getInterpolation()
                            .getFeedShotPitch(RobotState.getInstance().getDistancToFeedFromSource()));
        } else {
            shooter.setWantedState(
                    ShooterSubsystem.WantedState.FEED,
                    container
                            .getInterpolation()
                            .getFeedShotRPM(RobotState.getInstance().getDistanceToFeedTarget()),
                    container
                                    .getInterpolation()
                                    .getFeedShotRPM(RobotState.getInstance().getDistanceToFeedTarget())
                            * (6.0 / 11.0));
            pivot.setWantedState(
                    PivotSubsystem.WantedState.MOVE_TO_TARGET,
                    container
                            .getInterpolation()
                            .getFeedShotPitch(RobotState.getInstance().getDistanceToFeedTarget()));
        }
    }

    private void handleIntakeWhileInFeedingMode() {
        swerve.disableSlowdownCoefficients();
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
        feeder.setWantedState(FeederSubsystem.WantedState.FEED);
        if (RobotState.getInstance().withinChassisSpeeds(new ChassisSpeeds(0.2, 0.2, 4 * Math.PI))) {
            var trackpose = new Pose2d();
            if (container.getDashboard().getShouldFeedFromSource()) {
                trackpose = new Pose2d(
                        FieldConstants.getFeedShotFromSource().getX(),
                        FieldConstants.getFeedShotFromSource().getY()
                                + container.getDashboard().getFeedingYOffset(),
                        new Rotation2d());
            } else {
                trackpose = new Pose2d(
                        FieldConstants.getStationaryFeedShotTarget().getX(),
                        FieldConstants.getStationaryFeedShotTarget().getY()
                                + container.getDashboard().getFeedingYOffset(),
                        new Rotation2d());
            }
            trackFeedPoseWithTurret(trackpose);
            Logger.recordOutput("FeedShotTarget", trackpose);
            Logger.recordOutput("FeedShotMode", "Stationary");
        } else {
            var trackpose = new Pose2d();
            if (container.getDashboard().getShouldFeedFromSource()) {
                trackpose = new Pose2d(
                        FieldConstants.getFeedShotFromSource().getX(),
                        FieldConstants.getFeedShotFromSource().getY()
                                + container.getDashboard().getFeedingYOffset(),
                        new Rotation2d());
            } else {
                trackpose = new Pose2d(
                        FieldConstants.getRegularFeedShotTarget().getX(),
                        FieldConstants.getRegularFeedShotTarget().getY()
                                + container.getDashboard().getFeedingYOffset(),
                        new Rotation2d());
            }
            trackFeedPoseWithTurret(trackpose);
            Logger.recordOutput("FeedShotTarget", trackpose);
            Logger.recordOutput("FeedShotMode", "Regular");
        }
        if (container.getDashboard().getShouldFeedFromSource()) {
            shooter.setWantedState(
                    ShooterSubsystem.WantedState.FEED,
                    container
                            .getInterpolation()
                            .getFeedShotRPM(RobotState.getInstance().getDistancToFeedFromSource()),
                    container
                                    .getInterpolation()
                                    .getFeedShotRPM(RobotState.getInstance().getDistancToFeedFromSource())
                            * (6.0 / 11.0));
            pivot.setWantedState(
                    PivotSubsystem.WantedState.MOVE_TO_TARGET,
                    container
                            .getInterpolation()
                            .getFeedShotPitch(RobotState.getInstance().getDistancToFeedFromSource()));
        } else {
            shooter.setWantedState(
                    ShooterSubsystem.WantedState.FEED,
                    container
                            .getInterpolation()
                            .getFeedShotRPM(RobotState.getInstance().getDistanceToFeedTarget()),
                    container
                                    .getInterpolation()
                                    .getFeedShotRPM(RobotState.getInstance().getDistanceToFeedTarget())
                            * (6.0 / 11.0));
            pivot.setWantedState(
                    PivotSubsystem.WantedState.MOVE_TO_TARGET,
                    container
                            .getInterpolation()
                            .getFeedShotPitch(RobotState.getInstance().getDistanceToFeedTarget()));
        }
    }

    /** Utility states */
    private void homeAllPivots() {
        // turret.setWantedState(TurretSubsystem.WantedState.TARGET, new Rotation2d());
        turret.setWantedStateWithAbsoluteAngle(TurretSubsystem.WantedState.TARGET, new Rotation2d());
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        feeder.setWantedState(FeederSubsystem.WantedState.HOLD);
        intake.setWantedState(IntakeSubsystem.WantedState.REJECT);
        if (MathUtil.isNear(
                        0.0, RobotState.getInstance().getCurrentTurretAngle().getDegrees(), 5.0)
                && !climber.zeroCompleted()) {
            if (DriverStation.isTeleop()) {
                climber.setWantedState(ClimberSubsystem.WantedState.HOME);
            }
        }

        if (!pivot.zeroCompleted()) {
            pivot.setWantedState(PivotSubsystem.WantedState.HOME);
        }

        if (pivot.zeroCompleted() && climber.zeroCompleted()) {
            wantedSuperState = WantedSuperState.REGULAR_STATE;
        }
    }

    private void handleStopped() {
        swerve.disableSlowdownCoefficients();
        shooter.setWantedState(ShooterSubsystem.WantedState.OFF);
        pivot.setWantedState(PivotSubsystem.WantedState.IDLE);
        feeder.setWantedState(FeederSubsystem.WantedState.IDLE);
        intake.setWantedState(IntakeSubsystem.WantedState.IDLE);
    }

    private void handlePrepareSubwooferShot() {
        turret.setShouldVelocityCompensationBeApplied(false);
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        feeder.setWantedState(FeederSubsystem.WantedState.FEED);
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
        pivot.setWantedState(PivotSubsystem.WantedState.MOVE_TO_TARGET, Rotation2d.fromDegrees(57.0));
        turret.setWantedState(TurretSubsystem.WantedState.TARGET, Rotation2d.fromDegrees(180.0));
    }

    private void handleShootSubwooferShot() {
        turret.setShouldVelocityCompensationBeApplied(false);
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        feeder.setWantedState(FeederSubsystem.WantedState.SCORE);
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
        pivot.setWantedState(PivotSubsystem.WantedState.MOVE_TO_TARGET, Rotation2d.fromDegrees(57.0));
        turret.setWantedState(TurretSubsystem.WantedState.TARGET, Rotation2d.fromDegrees(180.0));
    }

    private void handleIntakeOuttake() {
        intake.setWantedState(IntakeSubsystem.WantedState.REJECT);
        shooter.setWantedState(ShooterSubsystem.WantedState.AMPING);
        trackSpeakerWithTurret();
        pivot.setWantedState(
                PivotSubsystem.WantedState.MOVE_TO_TARGET,
                container
                        .getInterpolation()
                        .getPitch(aimingParameters.effectiveDistance().getNorm()));
        feeder.setWantedState(FeederSubsystem.WantedState.IDLE);
    }

    private void handleOuttake() {
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
        turret.setWantedState(TurretSubsystem.WantedState.TARGET, new Rotation2d());
        pivot.setWantedState(PivotSubsystem.WantedState.MOVE_TO_TARGET, Rotation2d.fromDegrees(50.0));
        shooter.setWantedState(ShooterSubsystem.WantedState.AMPING);
        if (shooter.atOuttakeSetpoint() && turret.atSetpoint()) {
            feeder.setWantedState(FeederSubsystem.WantedState.SCORE);
        } else {
            feeder.setWantedState(FeederSubsystem.WantedState.FEED);
        }
    }

    /** Auto states */
    private void handlePrepareManualTurretAndPivot() {
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        feeder.setWantedState(FeederSubsystem.WantedState.FEED);
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
        pivot.setWantedState(PivotSubsystem.WantedState.MOVE_TO_TARGET, manualPitchSetpoint);
        turret.setWantedState(TurretSubsystem.WantedState.TARGET, manualTurretSetpoint);
    }

    private void handleShootManualTurretAndPivot() {
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        feeder.setWantedState(FeederSubsystem.WantedState.SCORE);
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
        pivot.setWantedState(PivotSubsystem.WantedState.MOVE_TO_TARGET, manualPitchSetpoint);
        turret.setWantedState(TurretSubsystem.WantedState.TARGET, manualTurretSetpoint);
    }

    private void handleMovePivotTo0() {
        pivot.setWantedState(PivotSubsystem.WantedState.MOVE_TO_TARGET, new Rotation2d());
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        feeder.setWantedState(FeederSubsystem.WantedState.FEED);
        turret.setWantedState(TurretSubsystem.WantedState.TARGET, aimingParameters.turretAimingAngle());
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
    }

    private void handleHomeOnlyPivot() {
        if (pivot.zeroCompleted()) {
            wantedSuperState = WantedSuperState.INTAKE_PIECE;
        } else {
            pivot.setWantedState(PivotSubsystem.WantedState.HOME);
        }
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        feeder.setWantedState(FeederSubsystem.WantedState.FEED);
        turret.setWantedState(TurretSubsystem.WantedState.TARGET, aimingParameters.turretAimingAngle());
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
    }

    private void handleAutoForceShot() {
        if (RobotState.getInstance().withinChassisSpeeds(new ChassisSpeeds(0.75, 0.75, 4 * Math.PI))) {
            trackSpeakerWithTurret();
        } else {
            turret.setWantedState(TurretSubsystem.WantedState.TARGET, aimingParameters.turretAimingAngle());
        }
        pivot.setWantedState(
                PivotSubsystem.WantedState.MOVE_TO_TARGET,
                container
                        .getInterpolation()
                        .getPitch(aimingParameters.effectiveDistance().getNorm()));
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        feeder.setWantedState(FeederSubsystem.WantedState.SCORE);
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
    }

    private void handleTrackWithoutPivot() {
        // trackSpeakerWithTurret();
        turret.setWantedState(TurretSubsystem.WantedState.TARGET, aimingParameters.turretAimingAngle());
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        feeder.setWantedState(FeederSubsystem.WantedState.FEED);
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
    }

    private void forceFeed() {
        feeder.setWantedState(FeederSubsystem.WantedState.SCORE);
    }

    private void handlePrepareSpeakerShot() {
        Logger.recordOutput(
                "WithinRegularTrackingSpeeds",
                RobotState.getInstance().withinChassisSpeeds(new ChassisSpeeds(0.75, 0.75, 4 * Math.PI)));
        if (RobotState.getInstance().withinChassisSpeeds(new ChassisSpeeds(0.75, 0.75, 4 * Math.PI))) {
            trackSpeakerWithTurret();
        } else {
            turret.setWantedState(TurretSubsystem.WantedState.TARGET, aimingParameters.turretAimingAngle());
        }
        intake.setWantedState(IntakeSubsystem.WantedState.COLLECT);
        pivot.setWantedState(
                PivotSubsystem.WantedState.MOVE_TO_TARGET,
                container
                        .getInterpolation()
                        .getPitch(aimingParameters.effectiveDistance().getNorm()));
        shooter.setWantedState(ShooterSubsystem.WantedState.SCORING);
        feeder.setWantedState(FeederSubsystem.WantedState.FEED);
    }

    /** Helper methods */
    private void trackSpeakerWithTurret() {
        Pose2d cameraPose = RobotState.getInstance().getSpeakerCameraPose();
        Logger.recordOutput("Tracking/CameraPose", cameraPose != null ? cameraPose : new Pose2d());

        if (cameraPose != null) {
            Rotation2d aprilTagAngle = RobotState.getInstance()
                    .getSpeakerCameraPose()
                    .getRotation()
                    .times(CompensationCoefficientConstants.REGULAR_TURRET_TRACKING_CAMERA_ANGLE_COEFFICIENT);

            Rotation2d turretAngleHasPose = RobotState.getInstance().getCurrentTurretAngle();
            Rotation2d targetTurretAngleHasPose = turretAngleHasPose.plus(aprilTagAngle);

            turret.setWantedState(TurretSubsystem.WantedState.TARGET, targetTurretAngleHasPose);

            Logger.recordOutput("Tracking/ThetaDegreesHasTarget", aprilTagAngle.getDegrees());
            Logger.recordOutput("Tracking/TargetTurretAngleHasTarget", targetTurretAngleHasPose.getDegrees());
            Logger.recordOutput("Tracking/AlphaDegreesHasTarget", turretAngleHasPose.getDegrees());
        } else {
            trackPoseWithTurret(FieldConstants.getSpeakerTag().toPose2d());
        }
    }

    public void trackPoseWithTurret(Pose2d target) {
        Pose2d pose = RobotState.getInstance().getOdometryPose();
        Rotation2d poseRotation = RobotState.getInstance().getOdometryPose().getRotation();
        Transform2d transform2d = new Transform2d(Units.inchesToMeters(-4.0), 0.0, new Rotation2d());

        Pose2d transformedPose = pose.plus(transform2d);

        double fiducialY = target.getY();
        double fiducialX = target.getX();

        double robotX = transformedPose.getX();
        double robotY = transformedPose.getY();

        double dY = fiducialY - robotY;
        double dX = FieldConstants.isBlueAlliance() ? fiducialX - robotX : robotX - fiducialX;

        Rotation2d arcTanAngle = Rotation2d.fromRadians(Math.atan(dY / dX));

        Rotation2d turretAngleTarget = new Rotation2d();

        if (FieldConstants.isBlueAlliance()) {
            turretAngleTarget = Rotation2d.fromDegrees(180).plus(arcTanAngle).minus(poseRotation);
        } else {
            turretAngleTarget = arcTanAngle.plus(poseRotation).unaryMinus();
        }

        turret.setWantedState(TurretSubsystem.WantedState.TARGET, turretAngleTarget);
    }

    public void trackFeedPoseWithTurret(Pose2d target) {
        Pose2d pose = RobotState.getInstance().getOdometryPose();
        Rotation2d poseRotation = RobotState.getInstance().getOdometryPose().getRotation();
        Transform2d transform2d = new Transform2d(Units.inchesToMeters(-4.0), 0.0, new Rotation2d());

        Pose2d transformedPose = pose.plus(transform2d);

        double fiducialY = target.getY();
        double fiducialX = target.getX();

        double robotX = transformedPose.getX();
        double robotY = transformedPose.getY();

        double dY = fiducialY - robotY;
        double dX = FieldConstants.isBlueAlliance() ? fiducialX - robotX : robotX - fiducialX;

        Rotation2d arcTanAngle = Rotation2d.fromRadians(Math.atan(dY / dX));

        var bounded = MathUtil.clamp(arcTanAngle.getDegrees(), -45.0, 0.0);
        // var targetAng = Math.copySign(bounded, arcTanAngle.getDegrees());
        Rotation2d boundedArcTanAngle = Rotation2d.fromDegrees(bounded);
        Logger.recordOutput("Feeding/Bounded Target Angle", bounded);
        Logger.recordOutput("Feeding/Original ArcTan Angle", arcTanAngle.getDegrees());
        Rotation2d turretAngleTarget = new Rotation2d();

        if (FieldConstants.isBlueAlliance()) {
            turretAngleTarget = Rotation2d.fromDegrees(180).plus(arcTanAngle).minus(poseRotation);
            boundedArcTanAngle =
                    Rotation2d.fromDegrees(180).plus(boundedArcTanAngle).minus(poseRotation);
        } else {
            turretAngleTarget = arcTanAngle.plus(poseRotation).unaryMinus();
            boundedArcTanAngle = boundedArcTanAngle.plus(poseRotation).unaryMinus();
        }

        Logger.recordOutput("Feeding/Bounded Actual Target", boundedArcTanAngle.getDegrees());
        Logger.recordOutput("Feeding/Original Actual Target", turretAngleTarget.getDegrees());

        turret.setWantedState(TurretSubsystem.WantedState.TARGET, boundedArcTanAngle);
    }

    public boolean hasPiece() {
        return feeder.isBeamBreakTripped();
    }

    public boolean pieceEntered() {
        return intake.pieceEntered();
    }

    public boolean isHomeFinished() {
        return pivot.zeroCompleted();
    }

    public CurrentSuperState getCurrentSuperState() {
        return currentSuperState;
    }

    public Rotation2d getClimbRotation() {
        Rotation2d heading = new Rotation2d();
        switch (container.getDashboard().getClimbLocation()) {
            case STAGE_FRONT -> heading =
                    FieldConstants.isBlueAlliance() ? Rotation2d.fromDegrees(0.0) : Rotation2d.fromDegrees(180.0);
            case STAGE_LEFT -> heading =
                    FieldConstants.isBlueAlliance() ? Rotation2d.fromDegrees(125.0) : Rotation2d.fromDegrees(-55.0);
            case STAGE_RIGHT -> heading =
                    FieldConstants.isBlueAlliance() ? Rotation2d.fromDegrees(-125.0) : Rotation2d.fromDegrees(55.0);
        }

        return heading;
    }

    public boolean getSubwooferShotMode() {
        return subwooferShotMode;
    }

    public boolean getFeedMode() {
        return feedShotMode;
    }

    public void toggleSubwooferShot() {
        subwooferShotMode = !subwooferShotMode;
    }

    public void toggleFeedMode() {
        feedShotMode = !feedShotMode;
    }

    /** State pushers */
    public void setWantedSuperState(WantedSuperState wantedSuperState) {
        this.wantedSuperState = wantedSuperState;
    }

    public Command setWantedSuperStateCommand(WantedSuperState wantedSuperState) {
        return new InstantCommand(() -> setWantedSuperState(wantedSuperState));
    }

    public Command setWantedSuperStateCommand(
            WantedSuperState wantedSuperState, Rotation2d manualTurretSetpoint, Rotation2d manualPitchSetpoint) {
        this.manualTurretSetpoint = manualTurretSetpoint;
        this.manualPitchSetpoint = manualPitchSetpoint;
        return new InstantCommand(() -> setWantedSuperState(wantedSuperState));
    }
}
