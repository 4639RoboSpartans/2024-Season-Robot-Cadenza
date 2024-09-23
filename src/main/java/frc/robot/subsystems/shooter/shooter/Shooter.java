package frc.robot.subsystems.shooter.shooter;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.InterpolatingTables;
import frc.robot.subsystems.shooter.constants.ShooterConstants;
import frc.robot.util.AimUtil;
import frc.robot.util.Helpers;

public class Shooter extends SubsystemBase implements IShooter {
    private final TalonFX shooterLeft, shooterRight;
    private final VelocityVoltage controlRequest;
    private final Trigger atSpeedTrigger;

    private static Shooter instance;

    private Shooter() {
        shooterLeft = new TalonFX(ShooterConstants.IDs.SHOOTER_SHOOTER_LEFT_MOTOR, "Canivore1");
        shooterRight = new TalonFX(ShooterConstants.IDs.SHOOTER_SHOOTER_RIGHT_MOTOR, "Canivore1");
        shooterLeft.setInverted(true);
        var leftConfigurator = shooterLeft.getConfigurator();
        var rightConfigurator = shooterRight.getConfigurator();
        var config = new TalonFXConfiguration()
                .withSlot0(
                        new Slot0Configs()
                                .withKP(
                                        0.1
                                ).withKV(
                                        0.0075
                                )
                );
        leftConfigurator.apply(config);
        rightConfigurator.apply(config);
        shooterRight.setControl(new Follower(shooterLeft.getDeviceID(), true));
        controlRequest = new VelocityVoltage(ShooterConstants.SHOOTER_IDLE_SPEED);
        atSpeedTrigger = new Trigger(
                () -> Helpers.withinTolerance(
                        shooterLeft.getVelocity().getValueAsDouble(),
                        controlRequest.Velocity,
                        ShooterConstants.SHOOTER_SPEED_ERROR
                )
        );
    }

    private void setSpeed(double speed) {
        shooterLeft.setControl(new VelocityVoltage(speed).withSlot(0));
    }

    @Override
    public Command autoShoot() {
        return runOnce(() -> setSpeed(
                InterpolatingTables.getSpeedTable().get(AimUtil.getSpeakerDist())));
    }

    @Override
    public Command manualShoot() {
        return runOnce(() -> setSpeed(
                InterpolatingTables.getSpeedTable().get(2.0)));
    }

    @Override
    public Command launch() {
        return runOnce(() -> setSpeed(
                ShooterConstants.SHOOTER_LAUNCH_SPEED
        ));
    }

    @Override
    public Command idle() {
        return runOnce(() -> shooterLeft.set(ShooterConstants.SHOOTER_IDLE_SPEED));
    }

    @Override
    public Command stop() {
        return runOnce(shooterLeft::stopMotor);
    }

    @Override
    public Trigger atSpeed() {
        return atSpeedTrigger;
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }
}
