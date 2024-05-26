// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ControllerRumbleCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.FlashLimelightCommand;
import frc.robot.commands.ManualTurretPointCommand;
import frc.robot.config.*;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOPhoenix6;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOPhoenix6;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOPhoenix6;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.pivot.*;
import frc.robot.subsystems.servo.ServoIOPhoenix6;
import frc.robot.subsystems.servo.ServoIOSim;
import frc.robot.subsystems.shooter.ShooterIOPhoenix6;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.*;
import frc.robot.subsystems.turret.CANCoderIO;
import frc.robot.subsystems.turret.CANCoderIOCANCoder;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import frc.robot.util.OperatorDashboard;
import frc.robot.util.ShooterInterpolation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final RobotState robotState = RobotState.getInstance();
    private SwerveSubsystem swerveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private VisionSubsystem visionSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private FeederSubsystem feederSubsystem;
    private PivotSubsystem pivotSubsystem;
    private TurretSubsystem turretSubsystem;
    private ClimberSubsystem climberSubsystem;
    private final XboxController primaryController = new XboxController(Constants.PRIMARY_XBOX_CONTROLLER_PORT);
    private final CommandXboxController simulatorController;

    private final OperatorDashboard dashboard;
    private final ShooterInterpolation interpolation;
    private final Superstructure superstructure;

    // Alerts
    private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        dashboard = new OperatorDashboard();
        interpolation = new ShooterInterpolation(dashboard);
        RobotConstants constants = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
        DrivetrainConfiguration swerveConfig = constants.getDrivetrainConfiguration();
        TurretConfiguration turretConfig = constants.getTurretConfiguration();
        LimelightConfiguration limelightConfig = constants.getLimelightConfiguration();

        switch (RobotIdentity.getMode()) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                swerveSubsystem = new SwerveSubsystem(
                        constants,
                        new GyroIOPigeon2(constants.getPigeon2Constants()),
                        new SwerveModuleIOPhoenix6(
                                swerveConfig.swerveModuleConfigurations[0],
                                constants.getDrivetrainConfiguration().SupportsPro),
                        new SwerveModuleIOPhoenix6(
                                swerveConfig.swerveModuleConfigurations[1],
                                constants.getDrivetrainConfiguration().SupportsPro),
                        new SwerveModuleIOPhoenix6(
                                swerveConfig.swerveModuleConfigurations[2],
                                constants.getDrivetrainConfiguration().SupportsPro),
                        new SwerveModuleIOPhoenix6(
                                swerveConfig.swerveModuleConfigurations[3],
                                constants.getDrivetrainConfiguration().SupportsPro));
                turretSubsystem = new TurretSubsystem(
                        new ServoIOPhoenix6(turretConfig.TurretServoMotorConfiguration),
                        turretConfig,
                        new CANCoderIOCANCoder(
                                turretConfig.CanCoderG1CanDeviceId,
                                turretConfig.EncodeOffsetG1Rotations,
                                turretConfig.g1Inverted),
                        new CANCoderIOCANCoder(
                                turretConfig.CanCoderG2CanDeviceId,
                                turretConfig.EncodeOffsetG2Rotations,
                                turretConfig.g2Inverted),
                        () -> climberSubsystem.zeroCompleted(),
                        () -> climberSubsystem.isStowed());
                shooterSubsystem = new ShooterSubsystem(new ShooterIOPhoenix6(constants.getPortConfiguration()), this);
                pivotSubsystem = new PivotSubsystem(
                        new PivotIOPhoenix6(constants.getPortConfiguration()),
                        new AmpBarIOPhoenix6(constants.getPortConfiguration()),
                        () -> climberSubsystem.isStowed(),
                        this);
                feederSubsystem = new FeederSubsystem(
                        new FeederIOPhoenix6(constants.getPortConfiguration()), () -> turretSubsystem.getTurretAngle());
                intakeSubsystem = new IntakeSubsystem(
                        new IntakeIOPhoenix6(constants.getPortConfiguration()),
                        () -> feederSubsystem.isBeamBreakTripped());
                visionSubsystem = new VisionSubsystem(new VisionIOLimelight(limelightConfig.Name), limelightConfig);
                climberSubsystem = new ClimberSubsystem(new ClimberIOPhoenix6(constants.getPortConfiguration()));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                swerveSubsystem = new SwerveSubsystem(
                        constants,
                        new GyroIO() {},
                        new SwerveModuleIOSim(swerveConfig.swerveModuleConfigurations[0]),
                        new SwerveModuleIOSim(swerveConfig.swerveModuleConfigurations[1]),
                        new SwerveModuleIOSim(swerveConfig.swerveModuleConfigurations[2]),
                        new SwerveModuleIOSim(swerveConfig.swerveModuleConfigurations[3]));
                visionSubsystem = new VisionSubsystem(new VisionIO() {}, limelightConfig);
                feederSubsystem = new FeederSubsystem(new FeederIOSim(), () -> Rotation2d.fromDegrees(0));
                intakeSubsystem = new IntakeSubsystem(new IntakeIOSim(), feederSubsystem::isBeamBreakTripped);
                pivotSubsystem = new PivotSubsystem(
                        new PivotIO() {}, new AmpBarIO() {}, () -> climberSubsystem.isStowed(), this);
                shooterSubsystem = new ShooterSubsystem(new ShooterIOSim() {}, this);
                turretSubsystem = new TurretSubsystem(
                        new ServoIOSim(turretConfig.TurretServoMotorConfiguration),
                        turretConfig,
                        new CANCoderIO() {},
                        new CANCoderIO() {},
                        () -> climberSubsystem.zeroCompleted(),
                        () -> climberSubsystem.isStowed());
                climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
                break;
            case REPLAY:
                // Replayed robot, disable IO implementations
                swerveSubsystem = new SwerveSubsystem(
                        constants,
                        new GyroIO() {},
                        new SwerveModuleIO() {},
                        new SwerveModuleIO() {},
                        new SwerveModuleIO() {},
                        new SwerveModuleIO() {});
                visionSubsystem = new VisionSubsystem(new VisionIO() {}, limelightConfig);
                feederSubsystem = new FeederSubsystem(new FeederIO() {}, () -> Rotation2d.fromDegrees(0));
                intakeSubsystem = new IntakeSubsystem(new IntakeIO() {}, () -> false);
                climberSubsystem = new ClimberSubsystem(new ClimberIO() {});
                break;
        }

        superstructure = new Superstructure(
                feederSubsystem,
                pivotSubsystem,
                shooterSubsystem,
                swerveSubsystem,
                turretSubsystem,
                intakeSubsystem,
                climberSubsystem,
                this);

        swerveSubsystem.setDefaultCommand(new DefaultDriveCommand(
                swerveSubsystem,
                () -> -primaryController.getLeftY(),
                () -> -primaryController.getLeftX(),
                () -> -primaryController.getRightX(),
                primaryController::getRightStickButton,
                primaryController::getRightBumper));

        if (Robot.isSimulation()) {
            simulatorController = new CommandXboxController(Constants.SIMULATOR_XBOX_CONTROLLER_PORT);
        } else {
            simulatorController = null;
        }

        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        new Trigger(primaryController::getLeftBumper)
                .onTrue(new ConditionalCommand(
                        superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.INTAKE_IN_FEED_MODE),
                        superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.INTAKE_PIECE),
                        superstructure::getFeedMode))
                .whileTrue(new WaitCommand(0.2)
                        .andThen(new ControllerRumbleCommand(
                                primaryController, () -> superstructure.hasPiece() || superstructure.pieceEntered())))
                .whileTrue(new WaitCommand(0.1)
                        .andThen(new FlashLimelightCommand(visionSubsystem)
                                .onlyIf(intakeSubsystem::pieceEntered)
                                .repeatedly()));
        new Trigger(primaryController::getLeftBumper)
                .onFalse(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE));

        new Trigger(primaryController::getXButton)
                .onTrue(superstructure.setWantedSuperStateCommand(
                        Superstructure.WantedSuperState.MOVE_CLIMBER_TO_STOW));

        new Trigger(primaryController::getYButton)
                .onTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.HOME_ALL_PIVOTS));

        new Trigger(primaryController::getAButton)
                .onTrue(new InstantCommand(superstructure::toggleFeedMode)
                        .andThen(superstructure.setWantedSuperStateCommand(
                                Superstructure.WantedSuperState.REGULAR_STATE)));

        new Trigger(primaryController::getYButton)
                .onTrue(new InstantCommand(() -> pivotSubsystem.setZeroCompleted(false)))
                .onTrue(new InstantCommand(() -> climberSubsystem.setZeroCompleted(false)));

        new Trigger(() -> primaryController.getLeftTriggerAxis() > 0.2)
                .onTrue(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.PREPARING_AMP_SHOT));
        new Trigger(() -> primaryController.getLeftTriggerAxis() > 0.2)
                .onFalse(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE));

        new Trigger(() -> primaryController.getRightTriggerAxis() > 0.2)
                .onTrue(superstructure.setWantedSuperStateCommand(
                        Superstructure.WantedSuperState.OUTTAKE_THROUGH_COLLECTOR))
                .onTrue(new FlashLimelightCommand(visionSubsystem))
                .onFalse(superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE));

        new Trigger(primaryController::getRightBumper)
                .onTrue(new ConditionalCommand(
                        superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.SHOOT_SUBWOOFER_SHOT),
                        new ConditionalCommand(
                                superstructure.setWantedSuperStateCommand(
                                        Superstructure.WantedSuperState.SHOOT_FEED_SHOT),
                                new ConditionalCommand(
                                        superstructure.setWantedSuperStateCommand(
                                                Superstructure.WantedSuperState.SHOOT_AMP),
                                        superstructure.setWantedSuperStateCommand(
                                                Superstructure.WantedSuperState.SHOOT_SPEAKER),
                                        () -> superstructure.getCurrentSuperState()
                                                        == Superstructure.CurrentSuperState.PREPARING_AMP_SHOT
                                                || superstructure.getCurrentSuperState()
                                                        == Superstructure.CurrentSuperState.SHOOTING_AMP),
                                () -> superstructure.getFeedMode() && primaryController.getLeftTriggerAxis() < 0.2),
                        superstructure::getSubwooferShotMode));

        new Trigger(primaryController::getRightBumper)
                .onFalse(new ConditionalCommand(
                        superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.PREPARING_AMP_SHOT),
                        superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.REGULAR_STATE),
                        () -> superstructure.getCurrentSuperState() == Superstructure.CurrentSuperState.SHOOTING_AMP));

        new Trigger(primaryController::getBackButton).onTrue(new InstantCommand(robotState::resetPose));
        new Trigger(primaryController::getStartButton)
                .onTrue(new InstantCommand(superstructure::toggleSubwooferShot)
                        .andThen(superstructure.setWantedSuperStateCommand(
                                Superstructure.WantedSuperState.REGULAR_STATE)));

        new Trigger(() -> Math.abs(primaryController.getRightX()) > 0.1)
                .whileTrue(new InstantCommand(() -> swerveSubsystem.disableRotationLock()).repeatedly());

        new Trigger(primaryController::getBButton)
                .onTrue(new ConditionalCommand(
                        superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.RETRACT_CLIMBER),
                        superstructure.setWantedSuperStateCommand(Superstructure.WantedSuperState.DEPLOY_CLIMBER),
                        () -> superstructure.getCurrentSuperState()
                                == Superstructure.CurrentSuperState.DEPLOY_CLIMBER));

        new Trigger(() -> primaryController.getPOV() == 0)
                .whileTrue(new ManualTurretPointCommand(turretSubsystem, primaryController));
        new Trigger(() -> primaryController.getPOV() == 90)
                .whileTrue(new ManualTurretPointCommand(turretSubsystem, primaryController));
        new Trigger(() -> primaryController.getPOV() == 180)
                .whileTrue(new ManualTurretPointCommand(turretSubsystem, primaryController));
        new Trigger(() -> primaryController.getPOV() == 270)
                .whileTrue(new ManualTurretPointCommand(turretSubsystem, primaryController));

        if (simulatorController != null) {
            simulatorController
                    .a()
                    .whileTrue(
                            Commands.startEnd(() -> robotState.setHasPiece(true), () -> robotState.setHasPiece(false)));
        }
    }

    public SwerveSubsystem getSwerveSubsystem() {
        return swerveSubsystem;
    }

    public Superstructure getSuperstructure() {
        return superstructure;
    }

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public TurretSubsystem getTurretSubsystem() {
        return turretSubsystem;
    }

    public PivotSubsystem getPivotSubsystem() {
        return pivotSubsystem;
    }

    public ClimberSubsystem getClimberSubsystem() {
        return climberSubsystem;
    }

    public ShooterInterpolation getInterpolation() {
        return interpolation;
    }

    public OperatorDashboard getDashboard() {
        return dashboard;
    }

    public XboxController getPrimaryController() {
        return primaryController;
    }

    /**
     * Updates the alerts for disconnected controllers.
     */
    public void checkControllers() {
        driverDisconnected.set(!DriverStation.isJoystickConnected(primaryController.getPort())
                || !DriverStation.getJoystickIsXbox(primaryController.getPort()));
    }
}
