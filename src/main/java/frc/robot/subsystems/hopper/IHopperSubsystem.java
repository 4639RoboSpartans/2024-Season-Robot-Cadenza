package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.sensors.IRSensor;

public interface IHopperSubsystem extends Subsystem {
    void run();
    void runBackwards();
    void stop();
    IRSensor getIR();
}
