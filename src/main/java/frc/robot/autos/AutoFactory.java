package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.commands.FollowPathCommand;
import frc.robot.config.FieldConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

/**
 * A factory for creating autonomous programs for a given {@link Auto}
 */
@SuppressWarnings({"UnusedMethod", "UnusedVariable", "EmptyBlockTag"})
class AutoFactory {
    private static final double AMPBAR_ZERO_DEGREES = 0.0;

    private final DriverStation.Alliance alliance;

    private final RobotContainer robotContainer;
    private final SwerveSubsystem swerve;
    private final Superstructure superstructure;
    private boolean trajectoriesLoaded = false;

    /**
     * Create a new <code>AutoFactory</code>.
     *
     * @param robotContainer The {@link RobotContainer}
     */
    AutoFactory(final DriverStation.Alliance alliance, final RobotContainer robotContainer) {
        this.alliance = alliance;
        this.robotContainer = robotContainer;
        this.swerve = robotContainer.getSwerveSubsystem();
        this.superstructure = robotContainer.getSuperstructure();
    }

    /* Autonomous program factories
     *
     * Factory methods should be added here for each autonomous program.
     * The factory methods must:
     *   1. Be package-private (i.e. no access modifier)
     *   2. Accept no parameters
     *   3. Return a link Command
     */
    private static final Command IDLE_COMMAND = Commands.idle();

    Command createIdleCommand() {
        return IDLE_COMMAND;
    }

    Command createAmpToBounceDEF() {
        var firstSegment = loadSegment(Location.AMP, Location.PREPLACE_D);

        preloadTrajectoryClass(firstSegment);

        SequentialCommandGroup c = new SequentialCommandGroup();

        c.addCommands(resetPose(firstSegment));
        c.addCommands(follow(firstSegment)
                .alongWith(shootPreloadWithHardcodedPivotAndTurret(
                        Rotation2d.fromDegrees(180.0), Rotation2d.fromDegrees(45.0))));
        c.addCommands(followThenShoot(Location.PREPLACE_D, Location.AMP_SHOT));
        c.addCommands(followWhileIntaking(Location.AMP_SHOT, Location.PREPLACE_E));
        c.addCommands(followThenShoot(Location.PREPLACE_E, Location.AMP_SHOT));
        c.addCommands(followWhileIntaking(Location.AMP_SHOT, Location.PREPLACE_F));
        c.addCommands(followThenShoot(Location.PREPLACE_F, Location.STAGE_SHOT));
        c.addCommands(followWhileIntaking(Location.STAGE_SHOT, Location.PREPLACE_E));
        return c;
    }

    Command createAmpToBounceDFE() {
        var firstSegment = loadSegment(Location.AMP, Location.PREPLACE_D);

        preloadTrajectoryClass(firstSegment);

        SequentialCommandGroup c = new SequentialCommandGroup();

        c.addCommands(resetPose(firstSegment));
        c.addCommands(follow(firstSegment)
                .alongWith(shootPreloadWithHardcodedPivotAndTurret(
                        Rotation2d.fromDegrees(180.0), Rotation2d.fromDegrees(45.0))));
        c.addCommands(followThenShoot(Location.PREPLACE_D, Location.AMP_SHOT));
        c.addCommands(followWhileIntaking(Location.AMP_SHOT, Location.PREPLACE_F));
        c.addCommands(followThenShoot(Location.PREPLACE_F, Location.AMP_SHOT));
        c.addCommands(followWhileIntaking(Location.AMP_SHOT, Location.PREPLACE_E));
        c.addCommands(followThenShoot(Location.PREPLACE_E, Location.AMP_SHOT));
        c.addCommands(followWhileIntaking(Location.AMP_SHOT, Location.PREPLACE_E));
        return c;
    }

    Command createSourceBounceHGF() {
        var firstSegment = loadSegment(Location.SOURCE, Location.PREPLACE_H);

        preloadTrajectoryClass(firstSegment);

        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(manualTurretOffsetTowardsGoal(15.0));
        c.addCommands(resetPose(firstSegment));
        c.addCommands(follow(firstSegment)
                .alongWith(superstructure
                        .setWantedSuperStateCommand(Superstructure.WantedSuperState.PREPARE_SPEAKER_SHOT)
                        .andThen(new WaitCommand(1.0)
                                .andThen(superstructure.setWantedSuperStateCommand(
                                        Superstructure.WantedSuperState.FORCE_SPEAKER_SHOT)))));
        c.addCommands(followThenShoot(Location.PREPLACE_H, Location.SOURCE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.SOURCE_SHOT, Location.PREPLACE_G));
        c.addCommands(followThenShoot(Location.PREPLACE_G, Location.SOURCE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.SOURCE_SHOT, Location.PREPLACE_F));
        c.addCommands(followThenShoot(Location.PREPLACE_F, Location.STAGE_SHOT_SOURCE, 0.2));
        c.addCommands(followWhileIntaking(Location.STAGE_SHOT, Location.PREPLACE_G));
        return c;
    }

    Command createSourceBounceFGH() {
        var firstSegment = loadSegment(Location.SOURCE, Location.PREPLACE_F);

        preloadTrajectoryClass(firstSegment);

        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(manualTurretOffsetTowardsGoal(30.0));
        c.addCommands(resetPose(firstSegment));
        c.addCommands(follow(firstSegment)
                .alongWith(superstructure
                        .setWantedSuperStateCommand(Superstructure.WantedSuperState.PREPARE_SPEAKER_SHOT)
                        .andThen(new WaitCommand(0.8)
                                .andThen(superstructure.setWantedSuperStateCommand(
                                        Superstructure.WantedSuperState.FORCE_SPEAKER_SHOT)))));
        c.addCommands(followThenShoot(Location.PREPLACE_F, Location.SOURCE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.SOURCE_SHOT, Location.PREPLACE_G));
        c.addCommands(followThenShoot(Location.PREPLACE_G, Location.SOURCE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.SOURCE_SHOT, Location.PREPLACE_H));
        c.addCommands(followThenShoot(Location.PREPLACE_H, Location.SOURCE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.SOURCE_SHOT, Location.PREPLACE_G));
        return c;
    }

    Command createSourceBounceGHF() {
        var firstSegment = loadSegment(Location.SOURCE, Location.PREPLACE_G);

        preloadTrajectoryClass(firstSegment);

        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(manualTurretOffsetTowardsGoal(15.0));
        c.addCommands(resetPose(firstSegment));
        c.addCommands(follow(firstSegment)
                .alongWith(superstructure
                        .setWantedSuperStateCommand(Superstructure.WantedSuperState.PREPARE_SPEAKER_SHOT)
                        .andThen(new WaitCommand(1.0)
                                .andThen(superstructure.setWantedSuperStateCommand(
                                        Superstructure.WantedSuperState.FORCE_SPEAKER_SHOT)))));
        c.addCommands(followThenShoot(Location.PREPLACE_G, Location.SOURCE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.SOURCE_SHOT, Location.PREPLACE_H));
        c.addCommands(followThenShoot(Location.PREPLACE_H, Location.SOURCE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.SOURCE_SHOT, Location.PREPLACE_F));
        c.addCommands(followThenShoot(Location.PREPLACE_F, Location.STAGE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.STAGE_SHOT, Location.PREPLACE_G));
        return c;
    }

    Command createSourceBounceHFG() {
        var firstSegment = loadSegment(Location.SOURCE, Location.PREPLACE_H);

        preloadTrajectoryClass(firstSegment);

        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(manualTurretOffsetTowardsGoal(15.0));
        c.addCommands(resetPose(firstSegment));
        c.addCommands(follow(firstSegment)
                .alongWith(superstructure
                        .setWantedSuperStateCommand(Superstructure.WantedSuperState.PREPARE_SPEAKER_SHOT)
                        .andThen(new WaitCommand(1.0)
                                .andThen(superstructure.setWantedSuperStateCommand(
                                        Superstructure.WantedSuperState.FORCE_SPEAKER_SHOT)))));
        c.addCommands(followThenShoot(Location.PREPLACE_H, Location.SOURCE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.SOURCE_SHOT, Location.PREPLACE_F));
        c.addCommands(followThenShoot(Location.PREPLACE_F, Location.SOURCE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.SOURCE_SHOT, Location.PREPLACE_G));
        c.addCommands(followThenShoot(Location.PREPLACE_G, Location.SOURCE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.SOURCE_SHOT, Location.PREPLACE_G));
        return c;
    }

    Command createSourceBounceFHG() {
        var firstSegment = loadSegment(Location.SOURCE, Location.PREPLACE_F);

        preloadTrajectoryClass(firstSegment);

        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(manualTurretOffsetTowardsGoal(15.0));
        c.addCommands(resetPose(firstSegment));
        c.addCommands(follow(firstSegment)
                .alongWith(superstructure
                        .setWantedSuperStateCommand(Superstructure.WantedSuperState.PREPARE_SPEAKER_SHOT)
                        .andThen(new WaitCommand(1.0)
                                .andThen(superstructure.setWantedSuperStateCommand(
                                        Superstructure.WantedSuperState.FORCE_SPEAKER_SHOT)))));
        c.addCommands(followThenShoot(Location.PREPLACE_F, Location.SOURCE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.SOURCE_SHOT, Location.PREPLACE_H));
        c.addCommands(followThenShoot(Location.PREPLACE_H, Location.SOURCE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.SOURCE_SHOT, Location.PREPLACE_G));
        c.addCommands(followThenShoot(Location.PREPLACE_G, Location.SOURCE_SHOT, 0.2));
        c.addCommands(followWhileIntaking(Location.SOURCE_SHOT, Location.PREPLACE_G));
        return c;
    }

    Command createSourceToCenterThenSweep() {
        var firstSegment = loadSegment(Location.SOURCE, Location.PREPLACE_F);

        preloadTrajectoryClass(firstSegment);

        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(manualTurretOffsetTowardsGoal(30.0));
        c.addCommands(resetPose(firstSegment));
        c.addCommands(follow(firstSegment)
                .alongWith(superstructure
                        .setWantedSuperStateCommand(Superstructure.WantedSuperState.PREPARE_SPEAKER_SHOT)
                        .andThen(new WaitCommand(0.8)
                                .andThen(superstructure.setWantedSuperStateCommand(
                                        Superstructure.WantedSuperState.FORCE_SPEAKER_SHOT)))));
        c.addCommands(followThenShoot(Location.PREPLACE_F, Location.STAGE_SHOT, 0.2));
        c.addCommands(follow(Location.STAGE_SHOT, Location.CENTER));
        c.addCommands(followThenShoot(Location.CENTER, Location.PREPLACE_B));
        c.addCommands(new WaitCommand(1.0));
        c.addCommands(follow(Location.PREPLACE_B, Location.PREPLACE_C)
                .alongWith(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.SHOOT_SPEAKER)));
        c.addCommands(new WaitCommand(1.0));
        c.addCommands(follow(Location.PREPLACE_C, Location.PREPLACE_A)
                .alongWith(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.SHOOT_SPEAKER)));
        return c;
    }

    private Command shootPreloadWithHardcodedPivotAndTurret(Rotation2d turret, Rotation2d pivot) {
        return Commands.sequence(
                superstructure.setWantedSuperStateCommand(
                        Superstructure.WantedSuperState.PREPARE_MANUAL_PIVOT_AND_TURRET, turret, pivot),
                Commands.waitSeconds(0.3),
                superstructure.setWantedSuperStateCommand(
                        Superstructure.WantedSuperState.SHOOT_MANUAL_PIVOT_AND_TURRET, turret, pivot),
                Commands.waitSeconds(0.3),
                superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.INTAKE_PIECE));
    }

    private Command followThenShoot(Location start, Location end) {
        return follow(start, end)
                .alongWith(
                        superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.AUTO_STATIONARY_SHOT))
                .andThen(new WaitUntilCommand(() -> !superstructure.hasPiece()));
    }

    private Command followThenShoot(Location start, Location end, double sec) {
        return follow(start, end)
                .alongWith(
                        superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.AUTO_STATIONARY_SHOT))
                .andThen(new WaitCommand(sec))
                .andThen(new WaitUntilCommand(() -> !superstructure.hasPiece()));
    }

    // Intake piece
    private Command followWhileIntaking(Location start, Location end) {
        return follow(start, end)
                .alongWith(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.INTAKE_PIECE));
    }

    // Auto init helpers
    private Command resetPose(final AutoSegment segment) {
        return runOnce(() -> {
            var pose = segment.path().getPreviewStartingHolonomicPose();
            RobotState.getInstance().reset(pose);
        });
    }

    private Command setTurret(Rotation2d rotation2d) {
        return Commands.runOnce(() ->
                robotContainer.getTurretSubsystem().setWantedState(TurretSubsystem.WantedState.TARGET, rotation2d));
    }

    private Command manualTurretOffsetTowardsGoal(double deg) {
        var angle = FieldConstants.isBlueAlliance() ? 180.0 + deg : 180.0 - deg;
        return setTurret(Rotation2d.fromDegrees(angle));
    }

    // Path following
    private FollowPathCommand follow(final Location start, final Location end) {
        return follow(loadSegment(start, end));
    }

    private FollowPathCommand follow(final AutoSegment segment) {
        return new FollowPathCommand(segment, swerve);
    }

    private void preloadTrajectoryClass(AutoSegment segment) {
        // This is done because Java loads classes lazily. Calling this here loads the trajectory class which
        // is used to follow paths and saves user code ms loop time at the start of auto.
        if (!trajectoriesLoaded) {
            trajectoriesLoaded = true;
            var trajectory = new PathPlannerTrajectory(
                    segment.path(),
                    RobotState.getInstance().getChassisSpeeds(),
                    RobotState.getInstance().getOdometryPose().getRotation());
        }
    }

    // Load paths
    private AutoSegment loadSegment(final Location start, final Location end) {
        var name = "%S_TO_%S".formatted(start, end);
        var path = PathPlannerPath.fromPathFile("%S_%S".formatted(alliance, name));

        return new AutoSegment(start, end, name, path);
    }
}
