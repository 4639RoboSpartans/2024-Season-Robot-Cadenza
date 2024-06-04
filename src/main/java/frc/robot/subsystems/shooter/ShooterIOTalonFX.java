package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.constants.IDs;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  StatusSignal<Double> shooterVoltage;
  StatusSignal<Double> shooterSpeed;

  public ShooterIOTalonFX() {
    leftTalon = new TalonFX(IDs.SHOOTER_SHOOTER_LEFT_MOTOR);
    rightTalon = new TalonFX(IDs.SHOOTER_SHOOTER_RIGHT_MOTOR);

    var shooterConfigLeft = new TalonFXConfiguration();
    shooterConfigLeft.CurrentLimits.SupplyCurrentLimit = 40.0;
    shooterConfigLeft.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfigLeft.CurrentLimits.withStatorCurrentLimit(4);
    leftTalon.getConfigurator().apply(shooterConfigLeft);
    leftTalon.setInverted(true);

    var shooterConfigRight = new TalonFXConfiguration();
    shooterConfigRight.CurrentLimits.SupplyCurrentLimit = 40.0;
    shooterConfigRight.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfigRight.CurrentLimits.withStatorCurrentLimit(4);
    rightTalon.getConfigurator().apply(shooterConfigRight);

    setBrakeMode(true);
    rightTalon.setControl(new Follower(IDs.SHOOTER_SHOOTER_LEFT_MOTOR, true));

    shooterVoltage = leftTalon.getMotorVoltage();
    shooterSpeed = leftTalon.getVelocity();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, shooterVoltage);

    leftTalon.optimizeBusUtilization();
    rightTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIO.ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(shooterVoltage, shooterSpeed);

    inputs.shooterVoltage = shooterVoltage.getValueAsDouble();
    inputs.shooterSpeed = shooterSpeed.getValueAsDouble();
  }

  @Override
  public void setShooterVoltage(double voltage) {
    leftTalon.setVoltage(voltage);
  }

  @Override
  public void set(double percent) {
    leftTalon.set(percent);
  }

  @Override
  public void setBrakeMode(boolean coast) {
    if (coast) {
      leftTalon.setNeutralMode(NeutralModeValue.Coast);
      rightTalon.setNeutralMode(NeutralModeValue.Coast);
    } else {
      leftTalon.setNeutralMode(NeutralModeValue.Brake);
      rightTalon.setNeutralMode(NeutralModeValue.Brake);
    }
  }

  @Override
  public void stop() {
    leftTalon.stopMotor();
  }
}
