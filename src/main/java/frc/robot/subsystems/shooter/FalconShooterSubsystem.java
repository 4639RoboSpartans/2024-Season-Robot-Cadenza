package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@SuppressWarnings("unused")
public class FalconShooterSubsystem extends SubsystemBase implements IShooterSubsystem {
    private final TalonFX shooterMotor;

    private final BangBangController bangBangController;

    private boolean isShooterRunning = false;

    public FalconShooterSubsystem(int shooterMotorID) {
        shooterMotor = new TalonFX(shooterMotorID);

        shooterMotor.setNeutralMode(NeutralModeValue.Coast);
        shooterMotor.setInverted(true);

        shooterMotor.getConfigurator().apply(
            new CurrentLimitsConfigs().withStatorCurrentLimit(3)
        );

        bangBangController = new BangBangController();
    }

    @Override
    public void periodic() {
        if(!isShooterRunning) {
            shooterMotor.stopMotor();
        }
        else {
            double currentSpeed = getCurrentSpeed();
            double targetSpeed = Constants.RobotInfo.TARGET_SHOOTER_SPEED;

            double speed = bangBangController.calculate(currentSpeed, targetSpeed);

            shooterMotor.set(speed * .5);
        }
    }

    private double getCurrentSpeed() {
        return shooterMotor.getVelocity().getValue();
    }

    @Override
    public boolean isUpToSpeed() {
        return Math.abs(getCurrentSpeed()) >= Math.abs(Constants.RobotInfo.TARGET_SHOOTER_SPEED) * 0.95;
    }

    public void runShooter() {
        isShooterRunning = true;
    }

    public void stopShooter() {
        isShooterRunning = false;
    }
}