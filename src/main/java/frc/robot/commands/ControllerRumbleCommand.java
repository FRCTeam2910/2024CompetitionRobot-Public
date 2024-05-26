package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class ControllerRumbleCommand extends Command {
    private XboxController controller;
    private BooleanSupplier condition;

    public ControllerRumbleCommand(XboxController controller, BooleanSupplier condition) {
        this.controller = controller;
        this.condition = condition;
    }

    @Override
    public void execute() {
        if (condition.getAsBoolean()) {
            controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.4);
        }
    }

    @Override
    public void end(boolean interrupted) {
        controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
    }
}
