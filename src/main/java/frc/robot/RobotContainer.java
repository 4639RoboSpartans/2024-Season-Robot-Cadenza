// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.shooter.*;
import frc.robot.constants.Controls;
import frc.robot.constants.Controls.DriverControls;
import frc.robot.constants.Controls.OperatorControls;
import frc.robot.commands.autos.AutoFactory;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.ManualClimbCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.intake.*;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.InterpolatingTables;
import frc.robot.led.LEDStrip;
import frc.robot.led.PhasingLEDPattern;
import frc.robot.led.SolidLEDPattern;
import frc.robot.oi.OI;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.climber.IClimberSubsystem;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.AimUtil;
import frc.robot.util.AutoHelper;
import frc.robot.util.DriverStationUtil;

@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class RobotContainer {
    public static OI oi;
    private final ISwerveDriveSubsystem swerveDriveSubsystem;

    private final IShooterSubsystem shooter;
    private final IIntakeSubsystem intake;
    private final IClimberSubsystem climber;
    private final IHopperSubsystem hopper;

    private final LEDStrip ledStrip;

    private final SendableChooser<Command> autos;
    private final SendableChooser<Double> delayTime;
    public static SendableChooser<Boolean> alliance;


    public RobotContainer() {
        InterpolatingTables.initializeTables();
        oi = new OI();
        ledStrip = SubsystemManager.getLedStrip();

        swerveDriveSubsystem = SubsystemManager.getSwerveDrive();

        shooter = SubsystemManager.getShooter();
        intake = SubsystemManager.getIntake();
        hopper = SubsystemManager.getHopper();
        climber = SubsystemManager.getClimber();

        nameCommands();

        autos = new SendableChooser<>();
        autos.setDefaultOption("null auto", new WaitCommand(1));
        autos.addOption("preloaded", AutoHelper.shoot());
        for (Command i : AutoFactory.getAutos()) {
            autos.addOption(i.getName(), i);
        }
        SmartDashboard.putData("Autons", autos);
        delayTime = new SendableChooser<Double>();
        delayTime.setDefaultOption("0", 0.0);
        delayTime.addOption("0", 0.0);
        delayTime.addOption("1", 1.0);
        delayTime.addOption("2", 2.0);
        delayTime.addOption("3", 3.0);
        delayTime.addOption("4", 4.0);
        SmartDashboard.putData(delayTime);

        alliance = new SendableChooser<>();
        alliance.addOption("Red", true);
        alliance.setDefaultOption("Blue", false);
        SmartDashboard.putData("Alliance", alliance);

        configureBindings();
    }

    private void nameCommands() {
        //climber commands
        NamedCommands.registerCommand("ExtendClimberCommand", new ExtendClimberCommand(climber));
        NamedCommands.registerCommand("ManualClimbCommand", new ManualClimbCommand(climber, 0, 0));
        NamedCommands.registerCommand("RetractClimberCommand", new RetractClimberCommand(climber));
        //intake commands
        NamedCommands.registerCommand("IntakeCommand", Commands.deadline(new WaitCommand(3), new IntakeCommand(intake, hopper, ledStrip, oi)));
        NamedCommands.registerCommand("OuttakeCommand", new OuttakeCommand(intake, hopper));
        NamedCommands.registerCommand("ExtendIntake", new ExtendIntakeCommand(intake));
        NamedCommands.registerCommand("RetractIntake", new RetractIntakeCommand(intake));

        // shooting commands
        NamedCommands.registerCommand("ShootSpeaker", new AutoSpeakerCommand(shooter, hopper, ledStrip));
        NamedCommands.registerCommand("ShootAmp", new AutoAmpCommand(intake));
        NamedCommands.registerCommand("ManualSpeaker", new ManualShootCommand(shooter, hopper, ledStrip));
    }


    private void configureBindings() {

        swerveDriveSubsystem.setDefaultCommand(
                swerveDriveSubsystem.driveFieldCentricCommand()
        );

        DriverControls.SOTF
                .whileTrue(
                        swerveDriveSubsystem.SOTFCommand()
                );

        DriverControls.AmpAlignButton
                .whileTrue(
                        Commands.parallel(
                                        swerveDriveSubsystem.pathfindCommand(
                                                new Pose2d(AimUtil.getAmpPose(), AimUtil.getAmpRotation())
                                        ),
                                        AutoHelper.ampPrepCommand()
                                )
                                .andThen(
                                        new AutoAmpCommand(intake)
                                )
                ).onFalse(
                        intake.runOnce(
                                intake::stopIntake
                        )
                );

        DriverControls.AimButton
                .whileTrue(
                        Commands.parallel(
                                swerveDriveSubsystem.pathfindCommand(
                                        AimUtil.getManualSpeakerPose()
                                ),
                                Commands.waitUntil(
                                        Controls.canSOTF
                                ).andThen(
                                        new AutoSpeakerCommand(shooter, hopper, ledStrip)
                                )
                        )
                );


        // TODO: extract to named class
        ledStrip.setDefaultCommand(new RunCommand(() -> {
            if (hopper.hasNote()) {
                ledStrip.usePattern(new PhasingLEDPattern(new Color8Bit(255, 50, 0), 3));
            } else {
                ledStrip.usePattern(new SolidLEDPattern(new Color8Bit(0, 0, 255)));
            }
        }, ledStrip));

        DriverControls.ClimberExtendButton.whileTrue(new ExtendClimberCommand(climber));
        DriverControls.ClimberRetractButton.whileTrue(new RetractClimberCommand(climber));
        DriverControls.ClimberSwap1Button.whileTrue(new ManualClimbCommand(climber, 1, -1));
        DriverControls.ClimberSwap2Button.whileTrue(new ManualClimbCommand(climber, -1, 1));

        OperatorControls.IntakeButton.whileTrue(intake.intake());

        OperatorControls.OuttakeButton.whileTrue(intake.outtake());

        OperatorControls.IntakeExtendButton.onTrue(intake.setExtended(IIntakeSubsystem.ExtensionState.EXTENDED));

        OperatorControls.IntakeRetractButton.onTrue(intake.setExtended(IIntakeSubsystem.ExtensionState.RETRACTED));

        OperatorControls.RunSpeakerShooterButton.whileTrue(new AutoSpeakerCommand(shooter, hopper, ledStrip));
        OperatorControls.RunAmpShooterButton.whileTrue(intake.setExtended(IIntakeSubsystem.ExtensionState.AMP).andThen(intake.amp()));
        OperatorControls.ManualShooterButton.whileTrue(new ManualShootCommand(shooter, hopper, ledStrip));
        OperatorControls.LaunchShooterButton.whileTrue(new LaunchCommand(shooter, hopper, ledStrip));

        OperatorControls.ToggleIR.onTrue(hopper.toggleIR());

        DriverControls.ResetGyroButton1.and(DriverControls.ResetGyroButton2).
                whileTrue(Commands.runOnce(swerveDriveSubsystem::reset));

        Controls.canSOTF
                .and(DriverControls.SOTF)
                .and(() -> !Robot.isInAuton())
                .whileTrue(AutoHelper.shoot());
        Controls.DriverControls.SOTF
                .and(Controls.canSOTF.negate())
                .and(() -> !Robot.isInAuton())
                .and(hopper::hasNote)
                .whileTrue(new ShooterSpinupCommand(shooter));
    }

    public Command getDelay() {
        return Commands.waitSeconds(delayTime.getSelected());
    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }
}