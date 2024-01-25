// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.ManualSwerveDriveCommand;
import frc.robot.commands.ShooterPivotCommand;
import frc.robot.oi.OI;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.climber.DummyClimberSubsystem;
import frc.robot.subsystems.climber.IClimberSubsystem;
import frc.robot.subsystems.intake.DummyIntakeSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.shooter.DummyShooterSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.shooterPivot.DummyShooterPivotSubsystem;
import frc.robot.subsystems.shooterPivot.IShooterPivotSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.tramp.DummyTrapSubsystem;
import frc.robot.subsystems.tramp.ITrapSubsystem;

@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class RobotContainer {
    private final OI oi;
    private final NavX navX;

    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private final IShooterSubsystem shooter;
    private final IShooterPivotSubsystem shooterPivot;
    private final IIntakeSubsystem intake;
    private final IClimberSubsystem climber;
    private final ITrapSubsystem tramp;


    public RobotContainer() {
        oi = new OI();
        navX = new NavX();

        swerveDriveSubsystem = new SwerveDriveSubsystem(navX);

        shooter = new DummyShooterSubsystem();
        shooterPivot = new DummyShooterPivotSubsystem();
        intake = new DummyIntakeSubsystem();
        climber = new DummyClimberSubsystem();
        tramp = new DummyTrapSubsystem();

        configureBindings();
    }


    private void configureBindings() {
        swerveDriveSubsystem.setDefaultCommand(new ManualSwerveDriveCommand(
                swerveDriveSubsystem, oi
        ));

        oi.getDriverController().getButton(OI.Buttons.Y_BUTTON).whileTrue(new RunCommand(navX::reset, navX));
        shooterPivot.setDefaultCommand(new ShooterPivotCommand(
                shooterPivot, oi
        ));;
    }


    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
