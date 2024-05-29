package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.aim.AimSubsystem;

public class ResetPIDCommand extends Command {
  private final AimSubsystem aimSubsystem;
  private boolean isReset = false;

  public ResetPIDCommand(AimSubsystem aimSubsystem) {
    this.aimSubsystem = aimSubsystem;
  }

  public void execute() {
    aimSubsystem.resetPID();
    isReset = true;
  }

  public boolean isFinished() {
    return isReset;
  }
}
