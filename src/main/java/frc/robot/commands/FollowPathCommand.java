package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.autos.AutoSegment;
import frc.robot.config.RobotConstants;
import frc.robot.config.RobotIdentity;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

public class FollowPathCommand extends Command {
    /**
     * Timer object
     */
    private final Timer timer = new Timer();

    /**
     * Drivetrain object to access subsystem.
     */
    private final SwerveSubsystem swerveSubsystem;

    /**
     * {@link Pair} to follow.
     */
    private final AutoSegment segment;

    /**
     * {@link PathPlannerTrajectory} to follow
     */
    private PathPlannerTrajectory trajectory;

    private static final PIDController rotationPID;

    private static final PIDController translationPID;

    public FollowPathCommand(final AutoSegment segment, SwerveSubsystem swerveSubsystem) {
        this.segment = segment;
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        rotationPID.reset();
        translationPID.reset();

        trajectory = new PathPlannerTrajectory(
                segment.path(),
                RobotState.getInstance().getChassisSpeeds(),
                RobotState.getInstance().getOdometryPose().getRotation());

        Logger.recordOutput("PathFollowing/PathName", segment.name());
        Logger.recordOutput(
                "PathFollowing/PerceivedOrientation",
                RobotState.getInstance().getOdometryPose().getRotation());
        this.timer.restart();
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        // Determine desired state based on where the robot should be at the current time in the path
        PathPlannerTrajectory.State desiredState = trajectory.sample(currentTime);
        var currentPose = RobotState.getInstance().getOdometryPose();

        Rotation2d heading = desiredState.heading;

        // Calculate our target velocity based on current pose and desired state
        var vx = desiredState.velocityMps * Math.cos(heading.getRadians());
        var vy = desiredState.velocityMps * Math.sin(heading.getRadians());
        var desiredThetaSpeeds = rotationPID.calculate(
                currentPose.getRotation().getRadians(), desiredState.targetHolonomicRotation.getRadians());

        double xFeedback = translationPID.calculate(
                currentPose.getX(), desiredState.getTargetHolonomicPose().getX());
        double yFeedback = translationPID.calculate(
                currentPose.getY(), desiredState.getTargetHolonomicPose().getY());

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                vx + xFeedback, vy + yFeedback, desiredThetaSpeeds, currentPose.getRotation());

        swerveSubsystem.setTargetSpeed(chassisSpeeds);

        Logger.recordOutput("PathFollowing/DesiredStatePose", desiredState.getTargetHolonomicPose());
        Logger.recordOutput("PathFollowing/DesiredChassisSpeeds", chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop(); // Stop timer
        swerveSubsystem.setTargetSpeed(new ChassisSpeeds()); // Stop motors
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    public Pose2d getStart() {
        return trajectory.getInitialState().getTargetHolonomicPose();
    }

    static {
        var configuration =
                RobotConstants.getRobotConstants(RobotIdentity.getIdentity()).getFollowPathConfiguration();

        translationPID = new PIDController(
                configuration.getTranslationKp(), configuration.getTranslationKi(), configuration.getTranslationKd());
        rotationPID = new PIDController(
                configuration.getRotationKp(), configuration.getRotationKi(), configuration.getRotationKd());
        rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    }
}
