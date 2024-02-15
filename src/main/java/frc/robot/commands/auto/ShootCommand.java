package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class ShootCommand extends Command {

    private final IShooterSubsystem shooter;

    ShootCommand(IShooterSubsystem shooter) {

        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.stopShooter();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }
}
