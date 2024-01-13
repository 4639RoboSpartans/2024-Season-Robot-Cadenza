// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ManualSwerveDriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.oi.OI;
import frc.robot.subsystems.climber.DummyClimberSubsystem;
import frc.robot.subsystems.climber.IClimberSubsystem;
import frc.robot.subsystems.intake.DummyIntakeSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.shooter.TwoWheelShooterNeos;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;
import frc.robot.subsystems.tramp.DummyTrampSubsystem;
import frc.robot.subsystems.tramp.ITrampSubsystem;


public class RobotContainer {
    private final OI oi;
    private final NavX navX;
    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private final IShooterSubsystem shooter;
    private final IIntakeSubsystem intakeSubsystem;
    private final IClimberSubsystem climberSubsystem;
    private final ITrampSubsystem trampSubsystem;


    public RobotContainer() {
        oi = new OI();
        navX = new NavX();

        swerveDriveSubsystem = new SwerveDriveSubsystem(navX);

        shooter = new TwoWheelShooterNeos(
                Constants.IDs.SHOOTER_MOTOR_LEFT,
                Constants.IDs.SHOOTER_MOTOR_RIGHT
        );
        intakeSubsystem = new DummyIntakeSubsystem();
        climberSubsystem = new DummyClimberSubsystem();
        trampSubsystem = new DummyTrampSubsystem();

        configureBindings();
    }
    
    
    private void configureBindings() {
        swerveDriveSubsystem.setDefaultCommand(new ManualSwerveDriveCommand(
            swerveDriveSubsystem, oi
        ));

        oi.getOperatorController().getButton(OI.Buttons.X_BUTTON).onTrue(
            new ShootCommand(shooter)
        );
    }
    
    
    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
