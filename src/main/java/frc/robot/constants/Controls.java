package frc.robot.constants;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.oi.OI;
import frc.robot.oi.OI.Buttons;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.util.AimUtil;

import java.util.function.DoubleSupplier;

public final class Controls {

    public static final class DriverControls {
        public static final DoubleSupplier SwerveForwardAxis = () -> {
            OI oi = SubsystemManager.getOI();
            return oi.driverController().getAxis(OI.Axes.LEFT_STICK_Y);
        };
        public static final DoubleSupplier SwerveStrafeAxis = () -> {
            OI oi = SubsystemManager.getOI();
            return -oi.driverController().getAxis(OI.Axes.LEFT_STICK_X);
        };
        public static final DoubleSupplier SwerveRotationAxis = () -> {
            OI oi = SubsystemManager.getOI();
            return -oi.driverController().getAxis(OI.Axes.RIGHT_STICK_X);
        };
        public static final Trigger AimButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.driverController().getButton(Buttons.LEFT_TRIGGER).getAsBoolean();
                });
        public static final Trigger SOTF = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.driverController().getButton(Buttons.RIGHT_TRIGGER).getAsBoolean();
                });
        public static final Trigger ClimberExtendButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.driverController().getButton(OI.Buttons.LEFT_BUMPER).getAsBoolean();
                });
        public static final Trigger ClimberRetractButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.driverController().getButton(OI.Buttons.RIGHT_BUMPER).getAsBoolean();
                });
        public static final Trigger ClimberSwap1Button = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.driverController().getButton(OI.Buttons.POV_LEFT).getAsBoolean();
                });
        public static final Trigger ClimberSwap2Button = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.driverController().getButton(OI.Buttons.POV_RIGHT).getAsBoolean();
                });

        public static final Trigger AmpAlignButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.driverController().getButton(OI.Buttons.X_BUTTON).getAsBoolean();
                });

        public static final Trigger ResetGyroButton1 = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.driverController().getButton(OI.Buttons.A_BUTTON).getAsBoolean();
                }),
                ResetGyroButton2 = new Trigger(
                        () -> {
                            OI oi = SubsystemManager.getOI();
                            return oi.driverController().getButton(OI.Buttons.B_BUTTON).getAsBoolean();
                        });
    }

    public static final class OperatorControls {
        public static final Trigger RunSpeakerShooterButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.operatorController().getButton(OI.Buttons.RIGHT_TRIGGER).getAsBoolean();
                });
        public static final Trigger RunAmpShooterButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.operatorController().getButton(OI.Buttons.LEFT_BUMPER).getAsBoolean();
                });
        public static final Trigger ManualShooterButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.operatorController().getButton(OI.Buttons.LEFT_TRIGGER).getAsBoolean();
                });

        public static final Trigger RunTrapShooterButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.operatorController().getButton(OI.Buttons.B_BUTTON).getAsBoolean();
                });

        public static final Trigger IntakeButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.operatorController().getButton(OI.Buttons.X_BUTTON).getAsBoolean();
                });
        public static final Trigger OuttakeButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.operatorController().getButton(OI.Buttons.Y_BUTTON).getAsBoolean();
                });
        public static final Trigger IntakeExtendButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.operatorController().getButton(OI.Buttons.POV_DOWN).getAsBoolean();
                });
        public static final Trigger IntakeRetractButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.operatorController().getButton(OI.Buttons.POV_UP).getAsBoolean();
                });
        public static final Trigger ShooterIntake = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.operatorController().getButton(OI.Buttons.POV_RIGHT).getAsBoolean();
                });

        public static final Trigger ToggleIR = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.operatorController().getButton(OI.Buttons.A_BUTTON).getAsBoolean();
                });

        public static final Trigger LaunchShooterButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.operatorController().getButton(OI.Buttons.RIGHT_BUMPER).getAsBoolean();
                });

        public static final Trigger FeedShooterButton = new Trigger(
                () -> {
                    OI oi = SubsystemManager.getOI();
                    return oi.operatorController().getButton(OI.Buttons.POV_LEFT).getAsBoolean();
                });
    }


    private static Trigger inShootingRange = new Trigger(AimUtil::inShootingRange);
    private static Trigger aligned = new Trigger(AimUtil::aligned);
    private static Trigger inShootingSector = new Trigger(AimUtil::inShootingSector);

    public static Trigger canSOTF = inShootingRange.and(aligned).and(inShootingSector);

    public static Trigger spinupTrigger = new Trigger(AimUtil::inRange);

    public static double rumbleStrength = 0.5;
}
