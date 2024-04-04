package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.sensors.IRSensor;

public class DummyHopperSubsystem extends SubsystemBase implements IHopperSubsystem {
    public void run(boolean checkNote, double speed) {}
    public void runBackwards(double speed) {}
    public void stop() {}
    public IRSensor getIR() { return null; }
    public void run(boolean checkNote, double speed, boolean reversed) {}
}
