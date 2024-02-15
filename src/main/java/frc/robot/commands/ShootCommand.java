package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class ShootCommand extends Command {
    private final IShooterSubsystem shooterSubsystem;

    public ShootCommand(IShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.stopShooter();
    }

    @Override
    public void execute() {
        shooterSubsystem.runShooter();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopShooter();
    }
}
