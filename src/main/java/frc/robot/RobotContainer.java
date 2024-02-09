// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.*;
import frc.robot.commands.auto.MoveCommand;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.ManualClimbCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.drive.ManualShooterPivotCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.semiauto.CenterLimelight;
import frc.robot.oi.OI;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.DummyClimberSubsystem;
import frc.robot.subsystems.climber.IClimberSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.DummyShooterSubsystem;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.shooterPivot.DummyShooterPivotSubsystem;
import frc.robot.subsystems.shooterPivot.IShooterPivotSubsystem;
import frc.robot.subsystems.swerve.AimSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class RobotContainer {
    private final OI oi;
    private final NavX navX;

    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private final IShooterSubsystem shooter;
    private final IShooterPivotSubsystem shooterPivot;
    private final IIntakeSubsystem intake;
    private final IClimberSubsystem climber;
    private final AimSubsystem aimSubsystem;
    private final IHopperSubsystem hopper;

    public RobotContainer() {
        oi = new OI();
        navX = new NavX();
        aimSubsystem = new AimSubsystem(oi);

        swerveDriveSubsystem = new SwerveDriveSubsystem(navX, aimSubsystem);

        shooter = new DummyShooterSubsystem();
        shooterPivot = new DummyShooterPivotSubsystem();
        intake = new IntakeSubsystem(Constants.IDs.INTAKE_PIVOT_MOTOR_LEFT, Constants.IDs.INTAKE_PIVOT_MOTOR_RIGHT, Constants.IDs.INTAKE_MOTOR, Constants.IDs.INTAKE_ENCODER);
        hopper = new HopperSubsystem(Constants.IDs.HOPPER_MOTOR);
        climber = new ClimberSubsystem(Constants.IDs.CLIMBER_LEFT, Constants.IDs.CLIMBER_RIGHT);

        configureBindings();
    }


    private void configureBindings() {
        swerveDriveSubsystem.setDefaultCommand(new ManualSwerveDriveCommand(
                swerveDriveSubsystem, oi
        ));

        shooterPivot.setDefaultCommand(new ManualShooterPivotCommand(
            shooterPivot, oi
        ));

        oi.getDriverController().getButton(Constants.Controls.Driver.ClimberExtendButton).whileTrue(new ExtendClimberCommand(climber));
        oi.getDriverController().getButton(Constants.Controls.Driver.ClimberRetractButton).whileTrue(new RetractClimberCommand(climber));
//        oi.getDriverController().getButton(Constants.Controls.Driver.ClimberSwap1Button).whileTrue(new ManualClimbCommand(climber, 1, -1));
//        oi.getDriverController().getButton(Constants.Controls.Driver.ClimberSwap2Button).whileTrue(new ManualClimbCommand(climber, 1, -1));


        oi.getDriverController().getButton(Constants.Controls.Driver.AimButton).onTrue(new ResetPIDCommand(swerveDriveSubsystem.getAimSubsystem()));

        oi.getOperatorController().getButton(Constants.Controls.Operator.ShooterButton).onTrue(new ShootCommand(shooter));
        oi.getOperatorController().getButton(Constants.Controls.Operator.IntakeButton).whileTrue(new IntakeCommand(intake, hopper));
        oi.getOperatorController().getButton(Constants.Controls.Operator.OuttakeButton).whileTrue(new OuttakeCommand(intake, hopper));

    }

    public Command getAutonomousCommand() {
        return new SequentialCommandGroup(
                new MoveCommand(swerveDriveSubsystem, .6, .3, 0, 2),
                new MoveCommand(swerveDriveSubsystem, -.5, 1, 0.5, 4)
        );
    }
}
