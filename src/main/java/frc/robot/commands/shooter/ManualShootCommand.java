package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RobotInfo.ShooterInfo.ShootingMode;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class ManualShootCommand extends Command{
    private final IShooterSubsystem shooterSubsystem;
    private final IHopperSubsystem hopperSubsystem;
    private boolean isShooting = false;

    public ManualShootCommand(IShooterSubsystem shooterSubsystem, IHopperSubsystem hopper){
        this.shooterSubsystem = shooterSubsystem;
        this.hopperSubsystem = hopper;

        addRequirements(shooterSubsystem, hopper);
    }

    @Override
    public void execute(){
        shooterSubsystem.setShootingMode(ShootingMode.SPEAKER);

        if (shooterSubsystem.isReady()){
            isShooting = true;
        }

        if (isShooting){
            hopperSubsystem.run();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShootingMode(ShootingMode.IDLE);
        hopperSubsystem.stop();
    }
}
