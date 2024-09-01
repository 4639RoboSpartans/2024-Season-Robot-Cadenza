package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Controls;
import frc.robot.led.LEDStrip;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class SOTFCommand extends SequentialCommandGroup {
    public SOTFCommand(IShooterSubsystem shooter, IHopperSubsystem hopper, LEDStrip led) {
        super(
                Commands.waitUntil(Controls.spinupTrigger),
                new ShooterSpinupCommand(shooter).until(Controls.canSOTF),
                new AutoSpeakerCommand(shooter, hopper, led)
        );
    }
}