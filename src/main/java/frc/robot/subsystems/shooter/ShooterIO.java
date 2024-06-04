package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double shooterVoltage;
    public double shooterSpeed;
  }

  /** sets shooter brake mode. */
  default void setBrakeMode(boolean coast) {}

  /** sets shooter voltage. */
  default void setShooterVoltage(double voltage) {}

  default void updateInputs(ShooterIOInputs inputs) {}

  /** sets percent speed. */
  default void set(double percent) {}

  default void stop() {}
}
