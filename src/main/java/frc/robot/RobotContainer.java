// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.MoveCommand;
import frc.robot.commands.semiauto.CenterLimelight;
import frc.robot.oi.OI;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.climber.DummyClimberSubsystem;
import frc.robot.subsystems.climber.IClimberSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.DummyShooterSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.shooterPivot.DummyShooterPivotSubsystem;
import frc.robot.subsystems.shooterPivot.IShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import static frc.robot.Constants.IDs.INTAKE_PIVOT_MOTOR_RIGHT;

@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class RobotContainer {
    private final OI oi;
    private final NavX navX;

    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private final IShooterSubsystem shooter;
    private final IShooterPivotSubsystem shooterPivot;
    private final IIntakeSubsystem intake;
    private final IClimberSubsystem climber;


    public RobotContainer() {
        oi = new OI();
        navX = new NavX();

        swerveDriveSubsystem = new SwerveDriveSubsystem(navX);

        shooter = new DummyShooterSubsystem();
        shooterPivot = new DummyShooterPivotSubsystem();
        intake = new IntakeSubsystem(Constants.IDs.INTAKE_PIVOT_MOTOR_LEFT, INTAKE_PIVOT_MOTOR_RIGHT, 0, Constants.IDs.INTAKE_ENCODER);
        climber = new DummyClimberSubsystem();

        configureBindings();
    }


    private void configureBindings() {
        swerveDriveSubsystem.setDefaultCommand(new ManualSwerveDriveCommand(
                swerveDriveSubsystem, oi
        ));

        oi.getDriverController().getButton(OI.Buttons.Y_BUTTON).whileTrue(new RunCommand(navX::reset, navX));
        
        shooterPivot.setDefaultCommand(new ManualShooterPivotCommand(
            shooterPivot, oi
        ));

        oi.getOperatorController().getButton(Constants.Controls.ShooterButton).onTrue(
            new ShootCommand(shooter)
        );

        oi.getOperatorController().getButton(Constants.Controls.ClimberExtendButton).onTrue(
            new ExtendClimberCommand(climber)
        );

        oi.getOperatorController().getButton(Constants.Controls.ClimberRetractButton).onTrue(
            new RetractClimberCommand(climber)
        );

        oi.getOperatorController().getButton(Constants.Controls.IntakeButton).whileTrue(
            new IntakeCommand(intake)
        );

        oi.getOperatorController().getButton(Constants.Controls.OuttakeButton).whileTrue(
            new OuttakeCommand(intake)
        );

        // TODO: use operator instead
        oi.getDriverController().getButton(Constants.Controls.LimeLightCenterButton).whileTrue(
            new CenterLimelight(swerveDriveSubsystem)
        );
    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                new MoveCommand(swerveDriveSubsystem, .6, .3, 0, 2),
                new MoveCommand(swerveDriveSubsystem, -.5, 1, 0.5, 4)
        );
    }
}
