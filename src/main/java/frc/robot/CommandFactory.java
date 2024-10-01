package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.CSM.CommandStateMachine;
import frc.lib.CSM.State;
import frc.lib.util.Helpers;
import frc.robot.constants.Controls;
import frc.robot.constants.DisplayInfo;
import frc.robot.led.LEDStrip;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.AimUtil;

public class CommandFactory {
    private static final SwerveDriveSubsystem swerve = SwerveDriveSubsystem.getInstance();
    private static final ShooterSuperstructure shooter = ShooterSuperstructure.getInstance();
    private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static final HopperSubsystem hopper = HopperSubsystem.getInstance();

    public static class Triggers {
        public static Trigger canSpinup = swerve.inShootingRange();
        public static Trigger canSOTF = canSpinup.and(swerve.inShootingSector()).and(swerve.isAligned());
    }

    public static Command intakeCommand() {
        return hopper.simToggleHasNote(false).andThen(intake.setExtended(IntakeSubsystem.ExtensionState.EXTENDED)
                .andThen(hopper.feed().alongWith(intake.intake()))
                .alongWith(Commands.run(() -> LEDStrip.getInstance().usePattern(DisplayInfo.intakePattern))));
    }

    public static Command shootCommand() {
        return Commands.repeatingSequence(
                        shooterIdleCommand().until(Triggers.canSpinup.and(hopper.hasNote())),
                        Commands.deadline(
                                pureShoot(),
                                Commands.sequence(
                                        Commands.waitUntil(Triggers.canSOTF).alongWith(
                                                Commands.run(
                                                        () -> LEDStrip.getInstance().usePattern(DisplayInfo.notReadyPattern))
                                        ),
                                        hopper.feed().alongWith(
                                                Commands.run(
                                                        () -> LEDStrip.getInstance().usePattern(DisplayInfo.readyPattern)
                                                )
                                        )
                                ).withTimeout(3)
                        ).until(Triggers.canSOTF.negate()))
                .andThen(hopper.simToggleHasNote(false));
    }

    public static Command pureShoot() {
        return shooter.runShootingMode(ShooterConstants.ShootingMode.AUTO_SPEAKER)
                .until(hopper.hasNote().negate());
    }

    public static Command manualShoot() {
        return shooter.runShootingMode(ShooterConstants.ShootingMode.MANUAL_SPEAKER)
                .until(hopper.hasNote().negate());
    }

    public static Command launchSpinup() {
        return shooter.runShootingMode(ShooterConstants.ShootingMode.LAUNCH);
    }

    public static Command launchCommand() {
        return launchSpinup().until(hopper.hasNote().negate());
    }

    public static Command shooterIdleCommand() {
        return shooter.runShootingMode(ShooterConstants.ShootingMode.IDLE);
    }

    public static Command ampPrepCommand() {
        return Commands.sequence(
                intake.setExtended(IntakeSubsystem.ExtensionState.EXTENDED),
                intake.outtake().alongWith(hopper.outtake()).until(hopper.hasNote().negate()),
                intake.setExtended(IntakeSubsystem.ExtensionState.RETRACTED)
        );
    }

    public static Command ampCommand() {
        return intake.amp();
    }

    public static Command outtakeCommand() {
        return Commands.sequence(
                hopper.simToggleHasNote(true),
                intake.setExtended(IntakeSubsystem.ExtensionState.RETRACTED),
                intake.outtake().alongWith(hopper.outtake())
        );
    }

    public static Command followPathCommand(String path) {
        return swerve.followChoreoPath(path, false);
    }

    public static Command aimCommand() {
        return swerve.SOTFCommand();
    }

    public static Command resetIntakeCommand() {
        return intake.stopIntake()
                        .andThen(
                                intake.setExtended(IntakeSubsystem.ExtensionState.RETRACTED)
                        );
    }

    public static Command getSuperstructureCSM() {
        CommandStateMachine superstructureStateMachine = new CommandStateMachine();

        State idleState = superstructureStateMachine.addState(
                shooterIdleCommand(),
                hopper.stop(),
                resetIntakeCommand());
        State hasNoteState = superstructureStateMachine.addState(
                shooterIdleCommand(),
                hopper.stop(),
                resetIntakeCommand());
        State SOTFState = superstructureStateMachine.addState(
                pureShoot(),
                hopper.feed(),
                resetIntakeCommand());
        State spinupState = superstructureStateMachine.addState(
                pureShoot(),
                hopper.stop(),
                resetIntakeCommand());
        State manualState = superstructureStateMachine.addState(
                manualShoot(),
                hopper.feed(),
                resetIntakeCommand());
        State launchSpinupState = superstructureStateMachine.addState(
                launchSpinup(),
                hopper.stop(),
                resetIntakeCommand());
        State launchState = superstructureStateMachine.addState(
                launchCommand(),
                hopper.feed(),
                resetIntakeCommand());
        State intakeState = superstructureStateMachine.addState(
                shooterIdleCommand(),
                hopper.feed(),
                intakeCommand());
        State outtakeState = superstructureStateMachine.addState(
                shooterIdleCommand(),
                hopper.outtake(),
                outtakeCommand());
        State ampPrep = superstructureStateMachine.addState(
                shooterIdleCommand(),
                ampPrepCommand());
        State ampReady = superstructureStateMachine.addState(
                shooterIdleCommand(),
                hopper.stop(),
                resetIntakeCommand());
        State amping = superstructureStateMachine.addState(
                shooterIdleCommand(),
                hopper.stop(),
                ampCommand());

        //define all transitions
        idleState.on(Controls.OperatorControls.IntakeButton, intakeState); // complete
        intakeState.on(Controls.OperatorControls.IntakeButton.negate(), idleState);

        intakeState.on(hopper.hasNote(), hasNoteState);

        hasNoteState.on(Controls.OperatorControls.OuttakeButton, outtakeState);
        outtakeState.on(Controls.OperatorControls.OuttakeButton.negate().and(hopper.hasNote().negate()), idleState);
        outtakeState.on(Controls.OperatorControls.OuttakeButton.negate().and(hopper.hasNote()), idleState);

        hasNoteState.on(swerve.inShootingRange().and(swerve.inShootingSector()).and(Controls.DriverControls.SOTF), spinupState);
        spinupState.on(Controls.DriverControls.SOTF.negate(), hasNoteState);

        spinupState.on(swerve.isAligned(), SOTFState);
        SOTFState.on(swerve.isAligned().negate(), spinupState);
        SOTFState.on(Controls.DriverControls.SOTF.negate(), hasNoteState);

        hasNoteState.on(Controls.OperatorControls.ManualShooterButton, manualState);
        manualState.on(Controls.OperatorControls.ManualShooterButton.negate(), hasNoteState);
        manualState.on(hopper.hasNote().negate(), idleState);

        hasNoteState.on(Controls.OperatorControls.RunAmpShooterButton, ampPrep);
        ampPrep.on(hopper.hasNote().negate(), ampReady);
        ampReady.on(() -> Helpers.withinTolerance(swerve.getPose().getTranslation().minus(AimUtil.getAmpPose()), 0.1), amping);

        amping.on(Controls.OperatorControls.RunAmpShooterButton.negate(), idleState);

        hasNoteState.on(Controls.OperatorControls.LaunchShooterButton.and(swerve.inLaunchRange().negate()), launchSpinupState);
        hasNoteState.on(Controls.OperatorControls.LaunchShooterButton.and(swerve.inLaunchRange()), launchState);
        launchSpinupState.on(swerve.inLaunchRange(), launchState);
        launchSpinupState.on(Controls.OperatorControls.LaunchShooterButton.negate(), hasNoteState);
        launchState.on(hopper.hasNote().negate(), idleState);

        superstructureStateMachine.setInitial(idleState);
        superstructureStateMachine.completeSetup();
        return superstructureStateMachine;
    }
}
