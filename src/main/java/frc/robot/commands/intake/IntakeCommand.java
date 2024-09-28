package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.led.LEDStrip;
import frc.robot.oi.OI;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.ConcreteIntakeSubsystem;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(ConcreteIntakeSubsystem intake, HopperSubsystem hopper, LEDStrip strip, OI oi) {
        super(new ExtendIntakeCommand(intake),
            new IntakeRollersCommand(intake, hopper, strip, oi),
            new RetractIntakeCommand(intake));
    }
}