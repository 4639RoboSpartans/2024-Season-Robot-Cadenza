package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.OI;
import frc.robot.oi.OI.Buttons;
import frc.robot.oi.OI.Axes;
import frc.robot.subsystems.swerve.SwerveConstants;

import java.util.function.DoubleSupplier;

public final class Controls {

    public static final class DriverControls {
        public static final DoubleSupplier SwerveForwardAxis = () -> {
            OI oi = OI.getInstance();
            return oi.driverController().getAxis(Axes.LEFT_STICK_Y) * SwerveConstants.CURRENT_MAX_ROBOT_MPS;
        };
        public static final DoubleSupplier SwerveStrafeAxis = () -> {
            OI oi = OI.getInstance();
            return -oi.driverController().getAxis(Axes.LEFT_STICK_X) * SwerveConstants.CURRENT_MAX_ROBOT_MPS;
        };
        public static final DoubleSupplier SwerveRotationAxis = () -> {
            OI oi = OI.getInstance();
            return -oi.driverController().getAxis(Axes.RIGHT_STICK_X);
        };
        public static final Trigger AimButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().getButton(Buttons.LEFT_TRIGGER).getAsBoolean();
                });
        public static final Trigger SOTF = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().getButton(Buttons.RIGHT_TRIGGER).getAsBoolean();
                });
        public static final Trigger ClimberExtendButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().getButton(Buttons.LEFT_BUMPER).getAsBoolean();
                });
        public static final Trigger ClimberRetractButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().getButton(Buttons.RIGHT_BUMPER).getAsBoolean();
                });
        public static final Trigger ClimberSwap1Button = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().getButton(Buttons.D_PAD_LEFT).getAsBoolean();
                });
        public static final Trigger ClimberSwap2Button = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().getButton(Buttons.D_PAD_RIGHT).getAsBoolean();
                });

        public static final Trigger AmpAlignButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().getButton(Buttons.X_BUTTON).getAsBoolean();
                });

        public static final Trigger ResetGyroButton1 = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.driverController().getButton(Buttons.A_BUTTON).getAsBoolean();
                }),
                ResetGyroButton2 = new Trigger(
                        () -> {
                            OI oi = OI.getInstance();
                            return oi.driverController().getButton(Buttons.B_BUTTON).getAsBoolean();
                        });
    }

    public static final class OperatorControls {
        public static final Trigger RunSpeakerShooterButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().getButton(Buttons.RIGHT_TRIGGER).getAsBoolean();
                });
        public static final Trigger RunAmpShooterButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().getButton(Buttons.LEFT_BUMPER).getAsBoolean();
                });
        public static final Trigger ManualShooterButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().getButton(Buttons.LEFT_TRIGGER).getAsBoolean();
                });

        public static final Trigger RunTrapShooterButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().getButton(Buttons.B_BUTTON).getAsBoolean();
                });

        public static final Trigger IntakeButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().getButton(Buttons.X_BUTTON).getAsBoolean();
                });
        public static final Trigger OuttakeButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().getButton(Buttons.Y_BUTTON).getAsBoolean();
                });
        public static final Trigger IntakeExtendButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().getButton(Buttons.D_PAD_DOWN).getAsBoolean();
                });
        public static final Trigger IntakeRetractButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().getButton(Buttons.D_PAD_UP).getAsBoolean();
                });
        public static final Trigger ShooterIntake = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().getButton(Buttons.D_PAD_RIGHT).getAsBoolean();
                });

        public static final Trigger ToggleIR = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().getButton(Buttons.A_BUTTON).getAsBoolean();
                });

        public static final Trigger LaunchShooterButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().getButton(Buttons.RIGHT_BUMPER).getAsBoolean();
                });

        public static final Trigger FeedShooterButton = new Trigger(
                () -> {
                    OI oi = OI.getInstance();
                    return oi.operatorController().getButton(Buttons.D_PAD_LEFT).getAsBoolean();
                });
    }
    public static double rumbleStrength = 0.5;
}
