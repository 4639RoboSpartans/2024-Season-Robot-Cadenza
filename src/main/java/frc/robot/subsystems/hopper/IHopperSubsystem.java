package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.sensors.IRTest;

public interface IHopperSubsystem extends Subsystem {
    void run();
    void runBackwards();
    void stop();
    IRTest getIR();
}
