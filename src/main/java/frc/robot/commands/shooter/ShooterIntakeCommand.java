package frc.robot.commands.shooter;

import frc.robot.constants.RobotInfo;
import frc.robot.led.LEDStrip;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.shooter.shooter.ShooterSubsystem;

public class ShooterIntakeCommand extends ShootCommand{
    public ShooterIntakeCommand(ShooterSubsystem shooter, HopperSubsystem hopper, LEDStrip ledStrip){
        super(shooter, hopper, ledStrip, RobotInfo.ShooterInfo.ShootingMode.INTAKE, true, false);
    }
}
