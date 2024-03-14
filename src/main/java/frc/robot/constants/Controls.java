package frc.robot.constants;

import frc.robot.oi.OI;

public final class Controls {

    public static final class DriverControls {
        public static final OI.Axes SwerveForwardAxis = OI.Axes.LEFT_STICK_Y;
        public static final OI.Axes SwerveStrafeAxis = OI.Axes.LEFT_STICK_X;
        public static final OI.Axes SwerveRotationAxis = OI.Axes.RIGHT_STICK_X;
        public static final OI.Buttons AimButton = OI.Buttons.LEFT_TRIGGER;

        public static final OI.Buttons ClimberExtendButton = OI.Buttons.LEFT_BUMPER;
        public static final OI.Buttons ClimberRetractButton = OI.Buttons.RIGHT_BUMPER;
        public static final OI.Buttons ClimberSwap1Button = OI.Buttons.POV_LEFT;
        public static final OI.Buttons ClimberSwap2Button = OI.Buttons.POV_RIGHT;

        public static final OI.Buttons AmpAlignButton = OI.Buttons.X_BUTTON;
    }

    public static final class OperatorControls {
        public static final OI.Buttons RunSpeakerShooterButton = OI.Buttons.RIGHT_TRIGGER;
        public static final OI.Buttons RunAmpShooterButton = OI.Buttons.LEFT_BUMPER;
        public static final OI.Buttons ManualShooterButton = OI.Buttons.LEFT_TRIGGER;

        public static final OI.Buttons RunTrapShooterButton = OI.Buttons.B_BUTTON;

        public static final OI.Buttons IntakeButton = OI.Buttons.X_BUTTON;
        public static final OI.Buttons OuttakeButton = OI.Buttons.Y_BUTTON;
        public static final OI.Buttons IntakeExtendButton = OI.Buttons.POV_DOWN;
        public static final OI.Buttons IntakeRetractButton = OI.Buttons.POV_UP;

        public static final OI.Buttons ToggleIR = OI.Buttons.A_BUTTON;

        public static final OI.Buttons LaunchShooterButton = OI.Buttons.RIGHT_BUMPER;

    }
}
