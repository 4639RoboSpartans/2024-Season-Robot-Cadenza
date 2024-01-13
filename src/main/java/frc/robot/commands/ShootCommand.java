package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class ShootCommand extends Command {
    private final IShooterSubsystem shooterSubsystem;

    public ShootCommand(IShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize() {
        shooterSubsystem.stop();
    }

    @Override
    public void execute() {
        shooterSubsystem.shoot(1.0);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stop();
    }
}
