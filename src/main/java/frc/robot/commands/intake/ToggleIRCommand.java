package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.sensors.IRSensor;

public class ToggleIRCommand extends Command {
  private final IRSensor irSensor;

  public ToggleIRCommand(IRSensor irSensor) {
    this.irSensor = irSensor;
  }

  @Override
  public void initialize() {
    // Toggle the IR sensor.
    irSensor.setActive(!irSensor.isActive());
  }
}
