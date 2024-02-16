package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IIntakeSubsystem;

public class SetIntakeExtendedCommand extends Command {
    private final IIntakeSubsystem intake;
    private final boolean extended;

    public SetIntakeExtendedCommand(IIntakeSubsystem intake, boolean extended) {
        this.intake = intake;
        this.extended = extended;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setExtended(extended);
    }
}
