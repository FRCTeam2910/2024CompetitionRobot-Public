package frc.robot.subsystems.swerve;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.*;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Threads;

public class Phoenix6Odometry {
    // CTRE Testing shows 1 (minimum realtime) is sufficient for tighter odometry loops.
    // If the odometry period is far away from the desired frequency, increasing this may help.
    AtomicBoolean isRunning = new AtomicBoolean(false);
    Thread thread;
    private static final int START_THREAD_PRIORITY = 1;
    protected final MedianFilter peakRemover = new MedianFilter(3);
    private final LinearFilter lowPass = LinearFilter.movingAverage(50);
    private int successfulDataAcquisitions = 0;
    private int failedDataAcquisitions = 0;
    private double currentTime = 0;
    private volatile double averageLoopTime = 0;
    private int lastThreadPriority = START_THREAD_PRIORITY;
    private int threadPriorityToSet = START_THREAD_PRIORITY;

    /** Prevents conflicts when reading signals */
    public ReadWriteLock stateLock = new ReentrantReadWriteLock();

    /** Prevents conflicts when registering signals */
    private final Lock signalsLock = new ReentrantLock();

    private BaseStatusSignal[] signals = new BaseStatusSignal[0];
    private boolean isCANFD = false;

    private double updateFrequency = 100.0;

    private static Phoenix6Odometry instance = null;

    /**
     * Get the singleton instance of the odometry thread.
     */
    public static Phoenix6Odometry getInstance() {
        if (instance == null) {
            instance = new Phoenix6Odometry();
        }
        return instance;
    }

    private Phoenix6Odometry() {
        thread = new Thread(this::run);
        thread.setName("Phoenix6OdometryThread");
        thread.setDaemon(true);
    }

    /**
     * Starts the odometry thread.
     */
    public void start() {
        if (!isRunning.getAndSet(true)) {
            thread.start();
        }
    }

    /**
     * Stops the odometry thread.
     */
    public void stop() {
        isRunning.getAndSet(false);
        try {
            thread.join();
        } catch (final InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Registers a signal for synchronized CAN IO on the odometry thread.
     * <p>The signal to register should set it's update frequency.</p>
     * @param device Device for signal to register.
     * @param signal Signal to register.
     */
    public void registerSignal(ParentDevice device, StatusSignal<Double> signal) {
        // TODO - probably want to check all devices are on same CAN bus
        isCANFD &= CANBus.isNetworkFD(device.getNetwork());
        signalsLock.lock();
        try {
            BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
            System.arraycopy(signals, 0, newSignals, 0, signals.length);
            newSignals[signals.length] = signal;
            signals = newSignals;
        } finally {
            signalsLock.unlock();
        }
    }

    /**
     * Background thread that refreshes all signals at the specified frequency.
     */
    public void run() {
        BaseStatusSignal.setUpdateFrequencyForAll(updateFrequency, signals);
        Threads.setCurrentThreadPriority(true, threadPriorityToSet);

        /* Run as fast as possible, the signals will control the timing */
        while (isRunning.get()) {
            StatusCode status = StatusCode.StatusCodeNotInitialized;
            signalsLock.lock();
            try {
                if (signals.length > 0) {
                    if (isCANFD) {
                        status = BaseStatusSignal.waitForAll(2.0 / updateFrequency, signals);
                    } else {
                        // "waitForAll" does not support blocking on multiple
                        // signals with a bus that is not CAN FD, regardless
                        // of Pro licensing. No reasoning for this behavior
                        // is provided by the documentation.
                        Thread.sleep((long) (1000.0 / updateFrequency));
                        if (signals.length > 0) {
                            status = BaseStatusSignal.refreshAll(signals);
                        }
                    }
                }
            } catch (InterruptedException e) {
                e.printStackTrace();
                Thread.currentThread().interrupt();
            } finally {
                signalsLock.unlock();
            }

            // TODO - do we even need the code below?
            if (status != StatusCode.StatusCodeNotInitialized) {
                stateLock.writeLock().lock();
                try {
                    var lastTime = currentTime;
                    currentTime = Utils.getCurrentTimeSeconds(); // HALUtil.getFPGATime() / 1.0e6;
                    /* We don't care about the peaks, as they correspond to GC events, and we want the period generally low passed */
                    averageLoopTime = lowPass.calculate(peakRemover.calculate(currentTime - lastTime));
                    if (status.isOK()) {
                        successfulDataAcquisitions++;
                    } else {
                        failedDataAcquisitions++;
                    }
                } finally {
                    stateLock.writeLock().unlock();
                }
            }

            if (threadPriorityToSet != lastThreadPriority) {
                Threads.setCurrentThreadPriority(true, threadPriorityToSet);
                lastThreadPriority = threadPriorityToSet;
            }
        }
    }

    /**
     * Check if the odometry is currently valid
     *
     * @return True if odometry is valid
     */
    @SuppressWarnings("unused")
    public boolean odometryIsValid() {
        stateLock.readLock().lock();
        try {
            return successfulDataAcquisitions > 2; // Wait at least 3 daqs before saying the odometry is valid
        } finally {
            stateLock.readLock().unlock();
        }
    }

    /**
     * Get the average loop time of the odometry thread.
     */
    @SuppressWarnings("unused")
    public double getTime() {
        stateLock.readLock().lock();
        try {
            return averageLoopTime;
        } finally {
            stateLock.readLock().unlock();
        }
    }

    /**
     * Get the number of successful data acquisition signals.
     */
    @SuppressWarnings("unused")
    public int getSuccessfulDataAcquisitions() {
        stateLock.readLock().lock();
        try {
            return successfulDataAcquisitions;
        } finally {
            stateLock.readLock().unlock();
        }
    }

    /**
     * Get the number of failed data acquisition signals.
     */
    @SuppressWarnings("unused")
    public int getFailedDataAcquisitions() {
        stateLock.readLock().lock();
        try {
            return failedDataAcquisitions;
        } finally {
            stateLock.readLock().unlock();
        }
    }

    @SuppressWarnings("unused")
    public int getThreadPriority() {
        return lastThreadPriority;
    }

    @SuppressWarnings("unused")
    public void setThreadPriority(int priority) {
        threadPriorityToSet = priority;
    }
}
