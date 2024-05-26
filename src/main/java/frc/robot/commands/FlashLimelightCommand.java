package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.VisionSubsystem;

public class FlashLimelightCommand extends Command {
    private final VisionSubsystem vision;
    private final Timer timer = new Timer();
    private static final double DURATION = 0.5;

    public FlashLimelightCommand(VisionSubsystem vision) {
        this.vision = vision;
    }

    @Override
    public void initialize() {
        timer.restart();
        vision.setLeds(true);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(DURATION);
    }

    @Override
    public void end(boolean interrupted) {
        vision.setLeds(false);
        timer.stop();
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
