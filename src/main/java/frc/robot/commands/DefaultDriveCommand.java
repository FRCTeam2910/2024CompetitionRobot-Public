package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DefaultDriveCommand extends Command {
    private static final double DEADBAND = 0.1;
    private static final double ROTATION_COEFFICIENT = 0.75;
    private final SwerveSubsystem drivetrain;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier angularSupplier;
    private final BooleanSupplier joystickButtonSupplier;
    private final BooleanSupplier isShootingSupplier;

    /**
     * The default command for the Drivetrain subsystem.  Accepts joystick inputs and passes
     * them to the drivetrain.
     *
     * @param swerveSubsystem        the Drivetrain subsystem.
     * @param xSupplier              The joystick input (Left Y).
     * @param ySupplier              The joystick input (Left X).
     * @param angularSupplier        The joystick (Right X).
     * @param joystickButtonSupplier Gets the joystick button input.
     */
    public DefaultDriveCommand(
            SwerveSubsystem swerveSubsystem,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier angularSupplier,
            BooleanSupplier joystickButtonSupplier,
            BooleanSupplier isShootingSupplier) {
        this.drivetrain = swerveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.angularSupplier = angularSupplier;
        this.joystickButtonSupplier = joystickButtonSupplier;
        this.isShootingSupplier = isShootingSupplier;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        double xMagnitude = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
        double yMagnitude = MathUtil.applyDeadband(ySupplier.getAsDouble(), DEADBAND);
        double angularMagnitude = MathUtil.applyDeadband(angularSupplier.getAsDouble(), DEADBAND);

        // Square the inputs to increase fine control while permitting full power.
        xMagnitude = Math.copySign(xMagnitude * xMagnitude, xMagnitude);
        yMagnitude = Math.copySign(yMagnitude * yMagnitude, yMagnitude);
        angularMagnitude = Math.copySign(angularMagnitude * angularMagnitude, angularMagnitude);

        // Calculate the x and y velocities in meters per second.
        double xVelocity = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? xMagnitude * drivetrain.getMaxVelocityMeterPerSecond()
                : -xMagnitude * drivetrain.getMaxVelocityMeterPerSecond();
        double yVelocity = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue
                ? yMagnitude * drivetrain.getMaxVelocityMeterPerSecond()
                : -yMagnitude * drivetrain.getMaxVelocityMeterPerSecond();
        double angularVelocity = angularMagnitude * drivetrain.getMaxAngularVelocityMeterPerSecond();

        // Scale angular velocity further if the button is pressed.
        angularVelocity *= joystickButtonSupplier.getAsBoolean() ? 1.0 : ROTATION_COEFFICIENT;

        ChassisSpeeds setpoint = ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                angularVelocity,
                RobotState.getInstance().getOdometryPose().getRotation());

        if (isShootingSupplier.getAsBoolean()) {
            double velocityVector = Math.hypot(
                    RobotState.getInstance().getChassisSpeeds().vxMetersPerSecond,
                    RobotState.getInstance().getChassisSpeeds().vyMetersPerSecond);
            double desiredVector = Math.hypot(xVelocity, yVelocity);
            if (desiredVector > velocityVector) {
                setpoint.times(velocityVector / desiredVector);
            }
        }

        drivetrain.setTargetSpeed(setpoint);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends.
        drivetrain.setTargetSpeed(new ChassisSpeeds());
    }
}
