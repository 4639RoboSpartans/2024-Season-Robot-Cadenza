package frc.robot.subsystems.shooter;

import static frc.robot.constants.RobotInfo.ShooterInfo;
import static frc.robot.constants.RobotInfo.ShooterInfo.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.shooter.pivot.IShooterPivotSubsystem;
import frc.robot.util.AimUtil;
import math.Averager;

public class FalconShooterSubsystem extends SubsystemBase implements IShooterSubsystem {
  // Components
  private final TalonFX shooterMotorLeft;
  private final TalonFX shooterMotorRight;
  private final IShooterPivotSubsystem shooterPivot;
  // Controllers
  private final Averager shooterOutputAverager;
  private final BangBangController bangBangController;
  // State
  private ShootingMode shootingMode;

  public FalconShooterSubsystem(int shooterMotorLeftID, int shooterMotorRightID) {
    this.shooterPivot = SubsystemManager.getShooterPivot(this);

    shooterMotorLeft = new TalonFX(shooterMotorLeftID);
    shooterMotorRight = new TalonFX(shooterMotorRightID);
    shooterMotorLeft.setNeutralMode(NeutralModeValue.Coast);
    shooterMotorRight.setNeutralMode(NeutralModeValue.Coast);

    shooterMotorLeft.setInverted(true);
    shooterMotorLeft.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(4));
    shooterMotorRight.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(4));
    shooterMotorRight.setControl(new Follower(shooterMotorLeftID, true));

    shooterOutputAverager = new Averager(Constants.POSE_WINDOW_LENGTH);

    bangBangController = new BangBangController();

    shootingMode = ShootingMode.IDLE;
  }

  private double getCurrentSpeed() {
    return shooterMotorLeft.getVelocity().getValue();
  }

  private double getTargetSpeed() {
    return switch (shootingMode) {
      case AUTO_SPEAKER -> AimUtil.getShooterSetpoint().speed();
      case SPEAKER -> SHOOTER_SPEAKER_SETPOINT.speed();
      case AMP -> SHOOTER_AMP_SETPOINT.speed();
      case TRAP -> SHOOTER_TRAP_SETPOINT.speed();
      case IDLE, INTAKE -> 0;
      case LAUNCH -> SHOOTER_LAUNCH_SETPOINT.speed();
    };
  }

  private void applyBangBangControl(double targetSpeed) {
    double currentSpeed = getCurrentSpeed();
    double controllerOutput = bangBangController.calculate(currentSpeed, targetSpeed);

    shooterOutputAverager.addMeasurement(controllerOutput);

    shooterMotorLeft.setVoltage(shooterOutputAverager.getValue() * ShooterInfo.SHOOTER_VOLTAGE);
  }

  private void applyIdleSpeed() {
    double speed = Robot.isInAuton() ? SHOOTER_AUTON_IDLE_SPEED : SHOOTER_IDLE_SPEED;
    shooterMotorLeft.set(speed);
  }

  private void applyIntakeSpeed() {
    shooterMotorLeft.set(ShooterInfo.SHOOTER_INTAKE_SPEED);
  }

  @Override
  public void periodic() {
    SmartDashboard.putString("shooting mode", shootingMode.toString());
    SmartDashboard.putNumber("shooter speed", getCurrentSpeed());
    SmartDashboard.putNumber("shooter target speed", getTargetSpeed());

    switch (shootingMode) {
      case AUTO_SPEAKER, SPEAKER, AMP, TRAP, LAUNCH -> applyBangBangControl(getTargetSpeed());
      case IDLE -> applyIdleSpeed();
      case INTAKE -> applyIntakeSpeed();
    }
  }

  public void setShootingMode(ShootingMode shooting) {
    shootingMode = shooting;
  }

  @Override
  public ShootingMode getShootingMode() {
    return shootingMode;
  }

  @Override
  public boolean isReady() {
    return isUpToSpeed() && shooterPivot.isAtSetPoint();
  }

  private boolean isUpToSpeed() {
    return Math.abs(getCurrentSpeed())
        >= Math.abs(getTargetSpeed())
            * switch (shootingMode) {
              case AMP -> .98;
              default -> 0.95;
            };
  }
}
