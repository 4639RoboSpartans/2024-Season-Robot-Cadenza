// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.autos.Autos;
import frc.robot.constants.Controls.DriverControls;
import frc.robot.constants.Controls.OperatorControls;
import frc.robot.constants.InterpolatingTables;
import frc.robot.led.LEDStrip;
import frc.robot.led.PhasingLEDPattern;
import frc.robot.led.SolidLEDPattern;
import frc.robot.oi.OI;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSuperstructure;
import frc.robot.subsystems.shooter.pivot.PivotSubsystem;
import frc.robot.subsystems.shooter.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.util.AimUtil;
import frc.robot.autos.AutoHelper;

@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class RobotContainer {
    public static OI oi;
    private final SwerveDriveSubsystem swerve;

    private final ShooterSuperstructure shooter;
    private final PivotSubsystem pivot;
    private final IntakeSubsystem intake;
    private final ClimberSubsystem climber;
    private final HopperSubsystem hopper;

    private final LEDStrip ledStrip;

    private final Mechanism2d robotMech;

    private final SendableChooser<Command> autos;
    private final SendableChooser<Double> delayTime;


    public RobotContainer() {
        InterpolatingTables.initializeTables();
        ledStrip = LEDStrip.getInstance();

        swerve = SwerveDriveSubsystem.getInstance();

        shooter = ShooterSuperstructure.getInstance();
        pivot = PivotSubsystem.getInstance();
        intake = IntakeSubsystem.getInstance();
        hopper = HopperSubsystem.getInstance();
        climber = ClimberSubsystem.getInstance();

        robotMech = new Mechanism2d(4, 4);

        pivot.initMech(robotMech);
        intake.initMech(robotMech);
        climber.initMech(robotMech);

        autos = new SendableChooser<>();
        autos.setDefaultOption("null auto", new WaitCommand(1));
        autos.addOption("preloaded", AutoHelper.shoot());
        autos.addOption("generated auto", Autos.getAutoCommand());
        SmartDashboard.putData("Autons", autos);
        delayTime = new SendableChooser<Double>();
        delayTime.setDefaultOption("0", 0.0);
        delayTime.addOption("0", 0.0);
        delayTime.addOption("1", 1.0);
        delayTime.addOption("2", 2.0);
        delayTime.addOption("3", 3.0);
        delayTime.addOption("4", 4.0);
        SmartDashboard.putData(delayTime);

        configureBindings();
    }


    private void configureBindings() {

        swerve.setDefaultCommand(
                swerve.driveFieldCentricCommand()
        );

        DriverControls.SOTF
                .whileTrue(
                        swerve.SOTFCommand())
                .onTrue(
                        CommandFactory.shootCommand())
                .onFalse(
                        hopper.simToggleHasNote(false));

        DriverControls.AmpAlignButton
                .whileTrue(
                        Commands.parallel(
                                        swerve.pathfindCommand(
                                                new Pose2d(AimUtil.getAmpPose(), AimUtil.getAmpRotation())
                                        ),
                                        CommandFactory.ampPrepCommand()
                                )
                                .andThen(
                                        CommandFactory.ampCommand()
                                )
                ).onFalse(
                        CommandFactory.resetIntakeCommand()
                );

        DriverControls.AimButton
                .whileTrue(
                        swerve.pathfindCommand(
                                AimUtil.getManualSpeakerPose()
                        ).andThen(
                                CommandFactory.pureShoot()))
                .onFalse(hopper.simToggleHasNote(false));

        DriverControls.SOTF.or(DriverControls.AimButton).negate().whileTrue(
                shooter.runShootingMode(ShooterConstants.ShootingMode.IDLE)
        );

        // TODO: extract to named class
        ledStrip.setDefaultCommand(new RunCommand(() -> {
            if (hopper.hasNote().getAsBoolean()) {
                ledStrip.usePattern(new PhasingLEDPattern(new Color8Bit(255, 50, 0), 3));
            } else {
                ledStrip.usePattern(new SolidLEDPattern(new Color8Bit(0, 0, 255)));
            }
        }, ledStrip));

        OperatorControls.IntakeButton.whileTrue(CommandFactory.intakeCommand())
                .onFalse(CommandFactory.resetIntakeCommand()
                        .andThen(hopper.simToggleHasNote(true)));

        OperatorControls.OuttakeButton.whileTrue(CommandFactory.outtakeCommand())
                .onFalse(CommandFactory.resetIntakeCommand()
                        .andThen(hopper.simToggleHasNote(false)));

        OperatorControls.IntakeExtendButton.onTrue(intake.setExtended(IntakeSubsystem.ExtensionState.EXTENDED));

        OperatorControls.IntakeRetractButton.onTrue(intake.setExtended(IntakeSubsystem.ExtensionState.RETRACTED));

        OperatorControls.ToggleIR.onTrue(hopper.toggleIR());

        DriverControls.ResetGyroButton1.and(DriverControls.ResetGyroButton2).
                whileTrue(Commands.runOnce(swerve::reset));
    }

    public Command getDelay() {
        return Commands.waitSeconds(delayTime.getSelected());
    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }

    public void sendSubsystems() {
        SmartDashboard.putData(swerve);
        SmartDashboard.putData(intake);
        SmartDashboard.putData(pivot);
        shooter.sendSubsystems();
        SmartDashboard.putData(hopper);
        SmartDashboard.putData("Robot Mechanism", robotMech);
    }
}