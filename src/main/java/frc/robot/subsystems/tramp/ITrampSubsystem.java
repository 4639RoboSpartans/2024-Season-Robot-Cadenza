package frc.robot.subsystems.tramp;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ITrampSubsystem extends Subsystem {
    void rotateArm(double degrees);

    void release();

    void setAngleDegrees(double degrees);

    void stop();

    void setHookAngleDegrees(double degrees);
}
