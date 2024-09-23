package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.OI;
import frc.robot.oi.OI.Axes;
import frc.robot.oi.OI.Buttons;

import java.util.function.DoubleSupplier;

import static frc.robot.subsystems.swerve.constants.SwerveConstants.CURRENT_MAX_ROBOT_MPS;
import static frc.robot.subsystems.swerve.constants.SwerveConstants.TELOP_ROTATION_SPEED;

public final class Controls {

    public static final class DriverControls {
        public static final DoubleSupplier SwerveForwardAxis = () -> {
            OI oi = OI.getInstance();
            return oi.driverController().axis(Axes.LEFT_STICK_Y) * CURRENT_MAX_ROBOT_MPS;
        };
        public static final DoubleSupplier SwerveStrafeAxis = () -> {
            OI oi = OI.getInstance();
            return -oi.driverController().axis(Axes.LEFT_STICK_X) * CURRENT_MAX_ROBOT_MPS;
        };
        public static final DoubleSupplier SwerveRotationAxis = () -> {
            OI oi = OI.getInstance();
            return -oi.driverController().axis(Axes.RIGHT_STICK_X) * TELOP_ROTATION_SPEED;
        };
        public static final Trigger AimButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().button(Buttons.LEFT_TRIGGER).getAsBoolean();
                });
        public static final Trigger SOTF = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().button(Buttons.RIGHT_TRIGGER).getAsBoolean();
                });
        public static final Trigger ClimberExtendButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().button(Buttons.LEFT_BUMPER).getAsBoolean();
                });
        public static final Trigger ClimberRetractButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().button(Buttons.RIGHT_BUMPER).getAsBoolean();
                });
        public static final Trigger ClimberSwap1Button = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().button(Buttons.D_PAD_LEFT).getAsBoolean();
                });
        public static final Trigger ClimberSwap2Button = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().button(Buttons.D_PAD_RIGHT).getAsBoolean();
                });

        public static final Trigger AmpAlignButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().button(Buttons.X_BUTTON).getAsBoolean();
                });

        public static final Trigger ResetGyroButton1 = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().button(Buttons.A_BUTTON).getAsBoolean();
                }),
                ResetGyroButton2 = new Trigger(
                        () -> {
                            OI oi = OI.getInstance();
                            return oi.driverController().button(Buttons.B_BUTTON).getAsBoolean();
                        });
    }

    public static final class OperatorControls {
        public static final Trigger RunSpeakerShooterButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().button(Buttons.RIGHT_TRIGGER).getAsBoolean();
                });
        public static final Trigger RunAmpShooterButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().button(Buttons.LEFT_BUMPER).getAsBoolean();
                });
        public static final Trigger ManualShooterButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().button(Buttons.LEFT_TRIGGER).getAsBoolean();
                });

        public static final Trigger IntakeButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().button(Buttons.X_BUTTON).getAsBoolean();
                });
        public static final Trigger OuttakeButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().button(Buttons.Y_BUTTON).getAsBoolean();
                });
        public static final Trigger IntakeExtendButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().button(Buttons.D_PAD_DOWN).getAsBoolean();
                });
        public static final Trigger IntakeRetractButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().button(Buttons.D_PAD_UP).getAsBoolean();
                });
        public static final Trigger ToggleIR = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().button(Buttons.A_BUTTON).getAsBoolean();
                });
        public static final Trigger LaunchShooterButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().button(Buttons.RIGHT_BUMPER).getAsBoolean();
                });

        public static final Trigger FeedShooterButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().button(Buttons.D_PAD_LEFT).getAsBoolean();
                });
    }
}
