package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DummyIntakeSubsystem extends SubsystemBase implements IIntakeSubsystem {
  @Override
  public void setExtended(ExtensionState degree) {}

  public void stop() {}

  public void intake() {}

  public void outtake() {}

  public void stopIntake() {}
}
