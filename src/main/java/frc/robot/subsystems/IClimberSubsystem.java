package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IClimberSubsystem extends Subsystem {
    void moveUp(double speed);
    void moveDown(double speed);
    void moveLeftUp(double speed);
    void moveLeftDown(double speed);
    void moveRightUp(double speed);
    void moveRightDown(double speed);
    void stop();
}
