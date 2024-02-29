package frc.robot.commands.shooter;

import frc.robot.Constants.RobotInfo.ShooterInfo.ShootingMode;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class AutoShootCommand extends ShootCommand {
    public AutoShootCommand(IShooterSubsystem shooter, IHopperSubsystem hopper) {
        super(shooter, hopper, null, ShootingMode.AUTO_SPEAKER);
    }
}