package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DummyHopperSubsystem extends SubsystemBase implements IHopperSubsystem {
    public void run(boolean checkNote, double speed) {}
    public void runBackwards(double speed) {}
    public void stop() {}
    public void run(boolean checkNote, double speed, boolean reversed) {}

    @Override
    public boolean hasNote() {
        return false;
    }

    @Override
    public Command toggleIR() {
        return null;
    }
}
