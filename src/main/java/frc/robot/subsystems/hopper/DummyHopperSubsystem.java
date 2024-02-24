package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DummyHopperSubsystem extends SubsystemBase implements IHopperSubsystem {
    public void run() {}
    public void runBackwards() {}
    public void stop() {}
    public boolean getIRSensor() { return false; }
}
