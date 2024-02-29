package frc.robot.commands.shooter;

import frc.robot.Constants.RobotInfo.ShooterInfo.ShootingMode;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class AutoAmpCommand extends ShootCommand {
    public AutoAmpCommand(IShooterSubsystem shooter, IHopperSubsystem hopper) {
        super(shooter, hopper, null, ShootingMode.AMP);
    }
}