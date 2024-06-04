package frc.robot.subsystems.shooter.pivot;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterPivotIOInputsAutoLogged extends ShooterPivotIO.ShooterPivotIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("PivotVoltage", pivotVoltage);
  }

  @Override
  public void fromLog(LogTable table) {
    pivotVoltage = table.get("PivotVoltage", pivotVoltage);
  }

  public ShooterPivotIOInputsAutoLogged clone() {
    ShooterPivotIOInputsAutoLogged copy = new ShooterPivotIOInputsAutoLogged();
    copy.pivotVoltage = this.pivotVoltage;
    return copy;
  }
}
