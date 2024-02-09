package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IHopperSubsystem extends Subsystem {
    void run();
    void runBackwards();
    void stop();
}
