package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climber.IClimberSubsystem;

public abstract class ClimbCommand extends Command {
  private final IClimberSubsystem climber;
  private final double leftSpeed;
  private final double rightSpeed;

  public ClimbCommand(IClimberSubsystem climber, double leftSpeed, double rightSpeed) {
    this.climber = climber;
    this.leftSpeed = leftSpeed;
    this.rightSpeed = rightSpeed;

    addRequirements(climber);
  }

  @Override
  public void execute() {
    climber.setLeftSpeed(leftSpeed);
    climber.setRightSpeed(rightSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }
}
