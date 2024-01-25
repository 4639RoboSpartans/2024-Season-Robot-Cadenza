package frc.robot.subsystems.tramp;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface ITrapSubsystem extends Subsystem {
    void rotateArm(double degrees);

    void release();

    void setAngleDegrees(double degrees);

    void stop();

    void intake();
}
