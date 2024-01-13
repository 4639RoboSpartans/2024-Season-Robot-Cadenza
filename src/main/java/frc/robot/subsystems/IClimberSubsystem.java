package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IClimberSubsystem extends Subsystem {
    void setSpeed(double speed);
    void setLeftSpeed(double speed);
    void setRightSpeed(double speed);
    void stop();
}
