package frc.robot.subsystems.swerve;

import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SubsystemManager;

public class GyroIONavX implements GyroIO {
  private final NavX navX;

  public GyroIONavX() {
    navX = SubsystemManager.getNavX();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = navX.getRotation2d();
    inputs.yawVelocityRadPerSec = navX.getRate();
  }
}
