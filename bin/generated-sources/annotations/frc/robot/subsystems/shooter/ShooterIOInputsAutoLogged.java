package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShooterIOInputsAutoLogged extends ShooterIO.ShooterIOInputs
    implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("ShooterVoltage", shooterVoltage);
    table.put("ShooterSpeed", shooterSpeed);
  }

  @Override
  public void fromLog(LogTable table) {
    shooterVoltage = table.get("ShooterVoltage", shooterVoltage);
    shooterSpeed = table.get("ShooterSpeed", shooterSpeed);
  }

  public ShooterIOInputsAutoLogged clone() {
    ShooterIOInputsAutoLogged copy = new ShooterIOInputsAutoLogged();
    copy.shooterVoltage = this.shooterVoltage;
    copy.shooterSpeed = this.shooterSpeed;
    return copy;
  }
}
