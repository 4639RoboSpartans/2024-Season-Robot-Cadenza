package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DisplayInfo;
import frc.robot.led.LEDStrip;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class CommandFactory {
    private static final SwerveDriveSubsystem swerve = SwerveDriveSubsystem.getInstance();
    private static final ShooterSuperstructure shooter = ShooterSuperstructure.getInstance();
    private static final IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private static final HopperSubsystem hopper = HopperSubsystem.getInstance();

    public static class Triggers {
        public static Trigger canSpinup = swerve.inShootingRange();
        public static Trigger canSOTF = canSpinup.and(swerve.inShootingSector()).and(swerve.isAligned()).and(shooter.atSetPoint());
    }

    public static Command intakeCommand() {
        return hopper.simToggleHasNote(false).andThen(intake.setExtended(IntakeSubsystem.ExtensionState.EXTENDED)
                .andThen(hopper.feed().alongWith(intake.intake()))
                .alongWith(Commands.run(() -> LEDStrip.getInstance().usePattern(DisplayInfo.intakePattern))));
    }

    public static Command shootCommand() {
        return Commands.sequence(
                        shooterIdleCommand().until(Triggers.canSpinup.and(hopper.hasNote())),
                        Commands.race(
                                autoSpinup(),
                                Commands.sequence(
                                        Commands.waitUntil(Triggers.canSOTF).alongWith(
                                                Commands.run(
                                                        () -> LEDStrip.getInstance().usePattern(DisplayInfo.notReadyPattern))
                                        ),
                                        hopper.feed().alongWith(
                                                Commands.run(
                                                        () -> LEDStrip.getInstance().usePattern(DisplayInfo.readyPattern)
                                                )
                                        ).deadlineWith(Commands.waitSeconds(3))
                                )
                        )
                ).andThen(hopper.simToggleHasNote(false));
    }

    public static Command autoSpinup() {
        return shooter.runShootingMode(ShooterConstants.ShootingMode.AUTO_SPEAKER)
                .until(hopper.hasNote().negate());
    }

    public static Command pureShoot() {
        return shooter.runShootingMode(ShooterConstants.ShootingMode.AUTO_SPEAKER)
                .alongWith(hopper.feed())
                .withTimeout(1)
                .until(() -> !hopper.hasNote().getAsBoolean());
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
        return swerve.followChoreoPath(path, true);
    }

    public static Command aimCommand() {
        return swerve.SOTFCommand();
    }

    public static Command resetIntakeCommand() {
        return intake.stopIntake().alongWith(hopper.stop())
                .andThen(
                        intake.setExtended(IntakeSubsystem.ExtensionState.RETRACTED)
                );
    }
}
