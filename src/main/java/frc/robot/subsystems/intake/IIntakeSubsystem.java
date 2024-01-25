package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IIntakeSubsystem extends Subsystem {
    void setIntakeDegree(double degrees);

    void stop();

    void intake();
}
