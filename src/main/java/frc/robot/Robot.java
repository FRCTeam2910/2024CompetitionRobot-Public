// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.autos.AutoChooser;
import frc.robot.subsystems.Superstructure;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;
    private AutoChooser autoChooser;
    private boolean isPivotAndClimberFakeHomed = false;

    private final LinearFilter average = LinearFilter.movingAverage(50);

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Record metadata
        Logger.recordMetadata("ProjectName", BuildInfo.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildInfo.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildInfo.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildInfo.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildInfo.GIT_BRANCH);
        switch (BuildInfo.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        // Set up data receivers & replay source
        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
        // Logger.disableDeterministicTimestamps()

        SignalLogger.enableAutoLogging(false);

        // Start AdvantageKit logger
        Logger.start();

        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();
        autoChooser = AutoChooser.create(robotContainer);
        Shuffleboard.getTab(Constants.OPERATOR_DASHBOARD_NAME)
                .add("Auto Program", autoChooser)
                .withSize(6, 3)
                .withPosition(12, 0)
                .withWidget(BuiltInWidgets.kComboBoxChooser);
        Shuffleboard.getTab(Constants.OPERATOR_DASHBOARD_NAME)
                .add(
                        "Fake Zero Pivot and Climber Command",
                        new InstantCommand(() -> {
                            robotContainer
                                    .getPivotSubsystem()
                                    .setAngle(Rotation2d.fromDegrees(0.601), Rotation2d.fromDegrees(0.0));
                            robotContainer.getClimberSubsystem().setAngle(new Rotation2d());
                            isPivotAndClimberFakeHomed = true;
                        }) {
                            @Override
                            public boolean runsWhenDisabled() {
                                return true;
                            }
                        });
        Shuffleboard.getTab(Constants.OPERATOR_DASHBOARD_NAME)
                .addBoolean("Pivot And Climber Fake Homed?", () -> isPivotAndClimberFakeHomed);
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.

        Threads.setCurrentThreadPriority(true, 99);

        double start = HALUtil.getFPGATime();
        CommandScheduler.getInstance().run();
        double end = HALUtil.getFPGATime();
        double commandSchedulerTime = (end - start) / 1e3;
        Logger.recordOutput("Times/CommandSchedulerMs", commandSchedulerTime);
        Logger.recordOutput("Times/CommandSchedulerMsAverage", average.calculate(commandSchedulerTime));

        Logger.recordOutput("Odometry/OdometryPose", RobotState.getInstance().getOdometryPose());
        Logger.recordOutput("Odometry/EstimatedPose", RobotState.getInstance().getEstimatedPose());
        Logger.recordOutput("Odometry/CameraPose", RobotState.getInstance().getSpeakerCameraPose());

        // Robot container periodic methods
        robotContainer.checkControllers();

        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(Constants.SILENCE_JOYSTICK_WARNINGS_IN_SIMULATOR);
        }

        Threads.setCurrentThreadPriority(true, 10);
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        autoChooser.reset("SmartDashboard/Auto/Programs");
    }

    @Override
    public void disabledPeriodic() {
        autoChooser.update();
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        robotContainer.getSwerveSubsystem().disableRotationLock();
        autoChooser.getSelectedCommand().ifPresent(CommandScheduler.getInstance()::schedule);
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        robotContainer.getSuperstructure().setWantedSuperState(Superstructure.WantedSuperState.REGULAR_STATE);
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        if (!robotContainer.getPrimaryController().getBackButton()) {
            RobotState.getInstance().resetPoseToVisionPose();
        }
        if (Math.abs(robotContainer.getPrimaryController().getRightX()) < 0.1) {
            if (robotContainer.getDashboard().getEnableRotationLock()) {
                robotContainer
                        .getSwerveSubsystem()
                        .setRotationLock(
                                RobotState.getInstance().getOdometryPose().getRotation());
            }
        } else {
            robotContainer.getSwerveSubsystem().disableRotationLock();
        }
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {}

    /**
     * This function is called once when the robot is first started up.
     */
    @Override
    public void simulationInit() {}

    /**
     * This function is called periodically whilst in simulation.
     */
    @Override
    public void simulationPeriodic() {}
}
