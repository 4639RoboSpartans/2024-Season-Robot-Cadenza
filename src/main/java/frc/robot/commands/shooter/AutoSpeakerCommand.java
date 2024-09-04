package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.RobotInfo;
import frc.robot.led.LEDStrip;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class AutoSpeakerCommand extends ShootCommand {
    public AutoSpeakerCommand(IShooterSubsystem shooter, IHopperSubsystem hopper, LEDStrip led) {
        super(shooter, hopper, led, RobotInfo.ShooterInfo.ShootingMode.AUTO_SPEAKER, false);
    }
}
