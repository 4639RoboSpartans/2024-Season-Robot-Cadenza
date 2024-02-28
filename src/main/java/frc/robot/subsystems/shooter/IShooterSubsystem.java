package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.RobotInfo.ShooterInfo.SHOOTING_MODE;

public interface IShooterSubsystem extends Subsystem {
    boolean isUpToSpeed();

    void runShooter();

    void stopShooter();

    void setShooting(SHOOTING_MODE shooting);
}