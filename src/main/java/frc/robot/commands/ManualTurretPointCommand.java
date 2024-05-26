package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ManualTurretPointCommand extends Command {
    private TurretSubsystem turretSubsystem;
    private XboxController controller;

    public ManualTurretPointCommand(TurretSubsystem turretSubsystem, XboxController controller) {
        this.turretSubsystem = turretSubsystem;
        this.controller = controller;

        addRequirements(turretSubsystem);
    }

    @Override
    public void execute() {
        turretSubsystem.setWantedState(
                TurretSubsystem.WantedState.TARGET, Rotation2d.fromDegrees(-controller.getPOV()));
    }

    @Override
    public void end(boolean interrupted) {
        turretSubsystem.setWantedState(TurretSubsystem.WantedState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return RobotState.getInstance().hasTarget();
    }
}
