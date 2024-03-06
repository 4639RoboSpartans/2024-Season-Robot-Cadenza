package frc.robot.commands.shooter;

import frc.robot.Constants;
import frc.robot.led.LEDStrip;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;

public class AutoTrapCommand extends ShootCommand{
    public AutoTrapCommand(IShooterSubsystem shooter, IHopperSubsystem hopper, LEDStrip ledStrip) {
        super(shooter, hopper, ledStrip, Constants.RobotInfo.ShooterInfo.ShootingMode.TRAP);
    }
}
