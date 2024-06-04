package frc.robot.subsystems.shooter.pivot;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import frc.robot.constants.IDs;

public class ShooterPivotIOSparkMax implements ShooterPivotIO {
  private final CANSparkMax pivotNeoLeft;
  private final CANSparkMax pivotNeoRight;

  public ShooterPivotIOSparkMax() {
    pivotNeoLeft = new CANSparkMax(IDs.SHOOTER_PIVOT_MOTOR_LEFT, CANSparkMax.MotorType.kBrushless);
    pivotNeoLeft.setIdleMode(CANSparkBase.IdleMode.kBrake);

    pivotNeoRight =
        new CANSparkMax(IDs.SHOOTER_PIVOT_MOTOR_RIGHT, CANSparkLowLevel.MotorType.kBrushless);
    pivotNeoRight.follow(pivotNeoLeft, true);
  }

  @Override
  public void set(double percent) {
    pivotNeoLeft.setVoltage(percent);
  }

  @Override
  public void updateInputs(ShooterPivotIOInputs inputs) {
    inputs.pivotVoltage = pivotNeoLeft.getBusVoltage();
  }

  @Override
  public void stop() {
    pivotNeoLeft.stopMotor();
  }
}
