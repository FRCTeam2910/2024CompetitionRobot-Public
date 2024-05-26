package frc.robot.subsystems.vision;

import java.util.EnumSet;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOLimelight implements VisionIO {
    private final NetworkTableEntry botposeEntry;
    private final NetworkTableEntry validEntry;
    private final NetworkTableEntry txEntry;
    private final NetworkTableEntry tyEntry;
    private final NetworkTableEntry tagIdEntry;
    private final NetworkTableEntry ledEntry;
    /**
     * Timestamp when last capture was taken.
     */
    private double lastTimeStamp = 0;
    /**
     * True if limelight sees targets.
     */
    private boolean hasTargets = false;
    /**
     * Horizontal error from center of limelight lens center to target
     * Units: Radians
     */
    private double yawRadians = 0;
    /**
     * Vertical error from center of limelight lens center to target
     */
    private double pitchRadians = 0;
    /**
     * TagID of target Fiducial marker.
     */
    private double tagId = 0;

    private double[] botpose = new double[6];

    public VisionIOLimelight(String limelightName) {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // Getting data from Network Tables
        validEntry = inst.getTable(limelightName).getEntry("tv");
        txEntry = inst.getTable(limelightName).getEntry("tx");
        tyEntry = inst.getTable(limelightName).getEntry("ty");
        botposeEntry = inst.getTable(limelightName).getEntry("botpose_wpiblue");
        tagIdEntry = inst.getTable(limelightName).getEntry("tid");
        ledEntry = inst.getTable(limelightName).getEntry("ledMode");

        // Getting LL latency values
        DoubleSubscriber pipelineLatencyEntrySub =
                inst.getTable(limelightName).getDoubleTopic("tl").subscribe(0.0);
        DoubleSubscriber captureLatencyEntrySub =
                inst.getTable(limelightName).getDoubleTopic("cl").subscribe(0.0);

        inst.addListener(pipelineLatencyEntrySub, EnumSet.of(NetworkTableEvent.Kind.kValueAll), networkTableEvent -> {
            // Compute in seconds
            var pipelineLatencies =
                    (pipelineLatencyEntrySub.getAtomic().value + captureLatencyEntrySub.getAtomic().value) / 1.0e3;
            double timeStamp = Timer.getFPGATimestamp() - pipelineLatencies;

            // Updating all inputs with values from LL.
            synchronized (VisionIOLimelight.this) {
                this.lastTimeStamp = timeStamp;
                hasTargets = validEntry.getDouble(0.0) == 1.0;
                yawRadians = -Units.degreesToRadians(txEntry.getDouble(0.0));
                pitchRadians = Units.degreesToRadians(tyEntry.getDouble(0.0));
                botpose = botposeEntry.getDoubleArray(new double[6]);
                tagId = tagIdEntry.getDouble(-1.0);
            }
        });
    }

    @Override
    public synchronized void updateInputs(VisionIOInputs inputs) {
        inputs.lastTimeStamp = this.lastTimeStamp;
        inputs.horizontalAngleRadians = this.yawRadians;
        inputs.verticalAngleRadians = this.pitchRadians;
        inputs.hasTargets = this.hasTargets;
        inputs.botpose = this.botpose;
        inputs.tagId = (int) this.tagId;
    }

    @Override
    public void setLeds(boolean on) {
        ledEntry.setNumber(on ? 2 : 1);
    }
}
