package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface IClimberSubsystem extends Subsystem {
  default void setSpeed(double speed) {
    setLeftSpeed(speed);
    setRightSpeed(speed);
  }

  void setLeftSpeed(double speed);

  void setRightSpeed(double speed);

  void stop();
}
