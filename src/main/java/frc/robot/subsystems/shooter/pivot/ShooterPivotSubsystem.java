package frc.robot.subsystems.shooter.pivot;

import static frc.robot.constants.RobotInfo.ShooterInfo;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IDs;
import frc.robot.constants.RobotInfo.ShooterInfo;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.util.AimUtil;

public class ShooterPivotSubsystem extends SubsystemBase implements IShooterPivotSubsystem {
  // Components
  private final ShooterPivotIO shooterPivotIO;
  private final ShooterPivotIOInputsAutoLogged shooterPivotInputs;

  private final DutyCycleEncoder encoder;
  // References to other subsystems
  private final IShooterSubsystem shooter;
  // Control
  private final PIDController aimPID;
  private boolean isUsingPID = true;

  public ShooterPivotSubsystem(IShooterSubsystem shooter, ShooterPivotIO shooterPivotIO) {
    this.shooterPivotIO = shooterPivotIO;
    this.shooterPivotInputs = new ShooterPivotIOInputsAutoLogged();
    encoder = new DutyCycleEncoder(IDs.SHOOTER_PIVOT_ENCODER_DIO_PORT);

    this.shooter = shooter;

    aimPID = ShooterInfo.SHOOTER_AIM_PID_CONSTANTS.create();
    aimPID.setSetpoint(ShooterInfo.SHOOTER_PIVOT_BOTTOM_SETPOINT);
  }

  public boolean isAtSetPoint() {
    double error = Math.abs(aimPID.getSetpoint() - encoder.getAbsolutePosition());
    return error < ShooterInfo.SHOOTER_PIVOT_ERROR;
  }

  @Override
  public void periodic() {
    shooterPivotIO.updateInputs(shooterPivotInputs);
    if (!isUsingPID) return;

    double targetAngle =
        switch (shooter.getShootingMode()) {
              case AUTO_SPEAKER -> AimUtil.getShooterSetpoint().angle();
              case SPEAKER -> ShooterInfo.SHOOTER_SPEAKER_SETPOINT.angle();
              case AMP -> ShooterInfo.SHOOTER_AMP_SETPOINT.angle();
              case TRAP -> ShooterInfo.SHOOTER_TRAP_SETPOINT.angle();
              case IDLE -> ShooterInfo.SHOOTER_PIVOT_BOTTOM_SETPOINT;
              case LAUNCH -> ShooterInfo.SHOOTER_LAUNCH_SETPOINT.angle();
              case INTAKE -> ShooterInfo.SHOOTER_INTAKE_SETPOINT.angle();
            }
            + ShooterInfo.AngleOffset;

    aimPID.setSetpoint(targetAngle);

    double currentAngle = getCurrentAngle();
    double pidOutput = aimPID.calculate(currentAngle);

    shooterPivotIO.set(pidOutput);

    SmartDashboard.putNumber("CurrentShooterAngle", currentAngle);
    SmartDashboard.putNumber("TargetShooterAngle", targetAngle);
    SmartDashboard.putNumber("AimPIDOutput", pidOutput);
  }

  private double getCurrentAngle() {
    return encoder.getAbsolutePosition();
  }

  public void stop() {
    shooterPivotIO.stop();
  }
}
