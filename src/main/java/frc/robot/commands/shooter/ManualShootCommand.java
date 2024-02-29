package frc.robot.commands.shooter;

import frc.robot.Constants.RobotInfo.ShooterInfo.ShootingMode;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class ManualShootCommand extends ShootCommand {
    public ManualShootCommand(IShooterSubsystem shooter, IHopperSubsystem hopper){
        super(shooter, hopper, null, ShootingMode.SPEAKER);
    }
}
