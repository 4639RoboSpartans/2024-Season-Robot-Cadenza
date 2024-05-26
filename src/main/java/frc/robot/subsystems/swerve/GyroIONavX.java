package frc.robot.subsystems.drive;

import frc.robot.subsystems.NavX;

public class GyroIONavX implements GyroIO {
  private final NavX navX;

  public GyroIONavX() {
    navX = new NavX();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = navX.getRotation2d();
    inputs.yawVelocityRadPerSec = navX.getRate();
  }
}
