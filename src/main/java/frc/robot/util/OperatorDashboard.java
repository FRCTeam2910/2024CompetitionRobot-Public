package frc.robot.util;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;

public class OperatorDashboard {
    ShuffleboardTab dashboard;
    private final SimpleWidget pitchOffset;
    private final SimpleWidget ampPivotPitchSlider;
    private final SimpleWidget ampRPMOffset;
    private final SimpleWidget feedRPMoffset;
    private final SimpleWidget shouldFeedFromSource;
    private final SimpleWidget rotationLock;
    private final SimpleWidget requireVisionForShot;
    private final SimpleWidget feedingYOffset;
    private final SendableChooser<ClimbLocation> climbLocation = new SendableChooser<>();

    public OperatorDashboard() {
        dashboard = Shuffleboard.getTab(Constants.OPERATOR_DASHBOARD_NAME);
        pitchOffset = dashboard
                .add("Pitch Offset", 0.0)
                .withSize(12, 3)
                .withPosition(0, 4)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -5.0, "max", 5.0, "block increment", 0.5));
        ampPivotPitchSlider = dashboard
                .add("Amp Pitch Offset", 48.0)
                .withSize(12, 3)
                .withPosition(0, 6)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 42.5, "max", 52.5, "block increment", 0.5));
        ampRPMOffset = dashboard
                .add("Amp RPM", 4500.0)
                .withSize(12, 3)
                .withPosition(13, 4)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 2000.0, "max", 8000.0, "block increment", 50));
        requireVisionForShot = dashboard
                .add("Require vision for shot?", true)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(0, 0)
                .withSize(3, 3);
        rotationLock = dashboard
                .add("Rotation Lock?", true)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(3, 0)
                .withSize(3, 3);
        feedingYOffset = dashboard
                .add("Feeding Y Offset", 0.0)
                .withSize(12, 3)
                .withPosition(4, 6)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -5.0, "max", 5.0, "block increment", 0.5));
        shouldFeedFromSource = dashboard
                .add("Should Feed from source", false)
                .withWidget(BuiltInWidgets.kToggleButton)
                .withPosition(5, 4)
                .withSize(3, 3);
        feedRPMoffset = dashboard
                .add("Feed offset RPM", 0.0)
                .withSize(3, 3)
                .withPosition(13, 6)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", -2000.0, "max", 2000.0, "block increment", 50));

        climbLocation.setDefaultOption("Stage Right", ClimbLocation.STAGE_RIGHT);
        climbLocation.addOption("Stage Left", ClimbLocation.STAGE_LEFT);
        climbLocation.addOption("Stage Front", ClimbLocation.STAGE_FRONT);

        dashboard
                .add("Climb Location", climbLocation)
                .withSize(6, 3)
                .withPosition(6, 0)
                .withWidget(BuiltInWidgets.kComboBoxChooser);
    }

    public double getPitchOffset() {
        return pitchOffset.getEntry().getDouble(0.0);
    }

    public double getAmpPitch() {
        return ampPivotPitchSlider.getEntry().getDouble(47.0);
    }

    public double getAmpRPM() {
        return ampRPMOffset.getEntry().getDouble(2800.0);
    }

    public boolean getRequireVisionForShot() {
        return requireVisionForShot.getEntry().getBoolean(true);
    }

    public boolean getEnableRotationLock() {
        return rotationLock.getEntry().getBoolean(true);
    }

    public double getFeedingYOffset() {
        return feedingYOffset.getEntry().getDouble(0.0);
    }

    public double getFeedRPMOffset() {
        return feedRPMoffset.getEntry().getDouble(0.0);
    }

    public boolean getShouldFeedFromSource() {
        return shouldFeedFromSource.getEntry().getBoolean(false);
    }

    public ClimbLocation getClimbLocation() {
        return climbLocation.getSelected();
    }

    public void incrementPitchOffset() {
        pitchOffset.getEntry().setDouble(pitchOffset.getEntry().getDouble(0.0) + 0.5);
    }

    public void decrementPitchOffset() {
        pitchOffset.getEntry().setDouble(pitchOffset.getEntry().getDouble(0.0) - 0.5);
    }
}
