package frc.robot.commands.climber;

import frc.robot.subsystems.climber.IClimberSubsystem;

public class ManualClimbCommand extends ClimbCommand {
    public ManualClimbCommand(IClimberSubsystem climber, double leftSpeed, double rightSpeed) {
        super(climber, leftSpeed, rightSpeed);
    }
}
