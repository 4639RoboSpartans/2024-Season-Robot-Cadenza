package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.IClimberSubsystem;

public class RetractClimberCommand extends ClimbCommand {
    public RetractClimberCommand(IClimberSubsystem climber) {
        super(climber, -1, -1);
    }
}