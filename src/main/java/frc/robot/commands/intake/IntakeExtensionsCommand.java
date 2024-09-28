package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.ExtensionState;

public abstract class IntakeExtensionsCommand extends Command {
    private final IntakeSubsystem intake;
    private final ExtensionState extended;

    private final double startTime = Timer.getFPGATimestamp();

    public IntakeExtensionsCommand(IntakeSubsystem intake, ExtensionState extended) {
        this.intake = intake;
        this.extended = extended;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setExtended(extended);
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
