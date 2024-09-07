package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Controls;
import frc.robot.constants.DisplayInfo;
import frc.robot.constants.RobotInfo.HopperInfo;
import frc.robot.led.LEDStrip;
import frc.robot.oi.OI;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem.ExtensionState;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(IIntakeSubsystem intake, IHopperSubsystem hopper, LEDStrip strip, OI oi) {
        super(new ExtendIntakeCommand(intake),
            new IntakeRollersCommand(intake, hopper, strip, oi));
    }
}