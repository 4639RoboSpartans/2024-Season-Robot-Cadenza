package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotInfo.ShooterInfo.ShootingMode;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class AutoAmpCommand extends Command {
    private final IShooterSubsystem shooter;
    private final IHopperSubsystem hopper;

    public AutoAmpCommand(IShooterSubsystem shooter, IHopperSubsystem hopper) {
        this.shooter = shooter;
        this.hopper = hopper;

        addRequirements(shooter, hopper);
    }

    @Override
    public void execute() {
        shooter.setShootingMode(ShootingMode.AMP);

        if(shooter.isReady()) {
            hopper.run(false);
        }
        else {
            hopper.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setShootingMode(ShootingMode.IDLE);
        hopper.stop();
    }
}