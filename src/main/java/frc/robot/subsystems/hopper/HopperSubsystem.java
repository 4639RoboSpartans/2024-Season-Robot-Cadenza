package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IHopperSubsystem extends Subsystem {
    void run(boolean checkNote, double speed);
    void runBackwards(double speed);
    void stop();
    void run(boolean checkNOte, double speed, boolean reversed);
    boolean hasNote();
    Command toggleIR();
}
