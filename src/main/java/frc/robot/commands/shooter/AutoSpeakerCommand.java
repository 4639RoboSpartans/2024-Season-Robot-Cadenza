package frc.robot.commands.shooter;

import frc.robot.constants.RobotInfo;
import frc.robot.led.LEDStrip;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class AutoSpeakerCommand extends ShootCommand {
    public AutoSpeakerCommand(IShooterSubsystem shooter, HopperSubsystem hopper, LEDStrip led) {
        super(shooter, hopper, led, RobotInfo.ShooterInfo.ShootingMode.AUTO_SPEAKER, false);
    }
}
