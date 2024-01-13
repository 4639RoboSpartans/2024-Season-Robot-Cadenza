package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DummyShooterSubsystem extends SubsystemBase implements IShooterSubsystem {
    @Override
    public void shoot(double speed) {}

    @Override
    public void stop() {}
}
