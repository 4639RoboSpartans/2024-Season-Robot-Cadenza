package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.RobotInfo.ShooterInfo.ShootingMode;

public interface IShooterSubsystem extends Subsystem {
    boolean isReady();

    void setShootingMode(ShootingMode shooting);

    ShootingMode getShootingMode();
}