package frc.robot.subsystems.shooter.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterPivotIO {
  @AutoLog
  class ShooterPivotIOInputs {
    public double pivotVoltage;
  }

  default void set(double percent) {}

  default void updateInputs(ShooterPivotIOInputs inputs) {}

  default void stop() {}
}
