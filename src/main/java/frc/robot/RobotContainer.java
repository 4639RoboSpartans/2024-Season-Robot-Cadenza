// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.ManualSwerveDriveCommand;
import frc.robot.Constants.Controls.DriverControls;
import frc.robot.Constants.Controls.OperatorControls;
import frc.robot.Constants.RobotInfo.ShooterInfo;
import frc.robot.commands.auto.ShootCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.commands.auto.MoveCommand;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.ManualClimbCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.intake.SetIntakeExtendedCommand;
import frc.robot.commands.semiauto.AutoAmpCommand;
import frc.robot.commands.semiauto.AutoShootCommand;
import frc.robot.oi.OI;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.DummyClimberSubsystem;
import frc.robot.subsystems.climber.IClimberSubsystem;
import frc.robot.subsystems.hopper.DummyHopperSubsystem;
import frc.robot.subsystems.hopper.HopperSubsystem;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.DummyIntakeSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.sensors.IRTest;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.shooter.DummyShooterSubsystem;
import frc.robot.subsystems.shooter.FalconShooterSubsystem;
import frc.robot.subsystems.shooterPivot.DummyShooterPivotSubsystem;
import frc.robot.subsystems.shooterPivot.IShooterPivotSubsystem;
import frc.robot.subsystems.shooterPivot.NeoShooterPivotSubsystem;
import frc.robot.subsystems.swerve.AimSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import static frc.robot.Constants.Controls.*;
import static frc.robot.Constants.RobotInfo.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

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
        
    private SendableChooser<Command> autos;
    private final IRTest ir;

    public RobotContainer() {
        oi = new OI();
        navX = new NavX();
        aimSubsystem = new AimSubsystem();
        ir = new IRTest();

        swerveDriveSubsystem = new SwerveDriveSubsystem(navX);

        swerveDriveSubsystem.resetOdometry(new Pose2d());

        autos = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autons", autos);

        shooter = switch(Constants.currentRobot) {
            case ZEUS -> new DummyShooterSubsystem();
            case SIREN -> new FalconShooterSubsystem(Constants.IDs.SHOOTER_SHOOTER_MOTOR, aimSubsystem);
        };
        shooterPivot = switch(Constants.currentRobot){
            case ZEUS -> new DummyShooterPivotSubsystem();
            case SIREN -> new NeoShooterPivotSubsystem(Constants.IDs.SHOOTER_PIVOT_MOTOR, aimSubsystem);
        };
        intake = switch(Constants.currentRobot){
            case ZEUS -> new DummyIntakeSubsystem();
            case SIREN -> new IntakeSubsystem(Constants.IDs.INTAKE_PIVOT_MOTOR_LEFT, Constants.IDs.INTAKE_PIVOT_MOTOR_RIGHT, Constants.IDs.INTAKE_MOTOR, Constants.IDs.INTAKE_ENCODER_CHANNEL);
        };
        hopper = switch(Constants.currentRobot){
            case ZEUS -> new DummyHopperSubsystem();
            case SIREN -> new HopperSubsystem(Constants.IDs.HOPPER_MOTOR, ir);
        };
        climber = switch(Constants.currentRobot){
            case ZEUS -> new DummyClimberSubsystem();
            case SIREN -> new ClimberSubsystem(Constants.IDs.CLIMBER_LEFT, Constants.IDs.CLIMBER_RIGHT);
        };

        //auto commands
        NamedCommands.registerCommand("MoveCommand", new MoveCommand(swerveDriveSubsystem, 0, 0, 0, 0));   
        NamedCommands.registerCommand("ShootCommand", new ShootCommand(shooter));   
        //climber commands
        NamedCommands.registerCommand("ExtendClimberCommand", new ExtendClimberCommand(climber));   
        NamedCommands.registerCommand("ManualClimbCommand", new ManualClimbCommand(climber, 0, 0));   
        NamedCommands.registerCommand("RetractClimberCommand", new RetractClimberCommand(climber));  
        //drive commands
        NamedCommands.registerCommand("ManualSwerveDriveCommand", new ManualSwerveDriveCommand(swerveDriveSubsystem, aimSubsystem, oi));   
        //intake commands
        NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(intake, hopper));   
        NamedCommands.registerCommand("OuttakeCommand", new OuttakeCommand(intake, hopper));   
        NamedCommands.registerCommand("SetIntakeExtendedCommand", new SetIntakeExtendedCommand(intake, false));   
        //semiauto commands
        NamedCommands.registerCommand("AutoShootCommand", new AutoShootCommand(shooter, shooterPivot, hopper));   
        


        configureBindings();
    }


    private void configureBindings() {
        swerveDriveSubsystem.setDefaultCommand(new ManualSwerveDriveCommand(
                swerveDriveSubsystem, aimSubsystem, oi
        ));

        oi.driverController().getButton(DriverControls.ClimberExtendButton).whileTrue(new ExtendClimberCommand(climber));
        oi.driverController().getButton(DriverControls.ClimberRetractButton).whileTrue(new RetractClimberCommand(climber));
        oi.driverController().getButton(DriverControls.ClimberSwap1Button).whileTrue(new ManualClimbCommand(climber, 1, -1));
        oi.driverController().getButton(DriverControls.ClimberSwap2Button).whileTrue(new ManualClimbCommand(climber, -1, 1));
        oi.operatorController().getButton(OperatorControls.IntakeButton).whileTrue(new IntakeCommand(intake, hopper));
 
        oi.operatorController().getButton(OperatorControls.OuttakeButton).whileTrue(new OuttakeCommand(intake, hopper));

        oi.operatorController().getButton(OperatorControls.IntakeExtendButton).whileTrue(new SetIntakeExtendedCommand(intake, true));

        oi.operatorController().getButton(OperatorControls.IntakeRetractButton).onTrue(new SetIntakeExtendedCommand(intake, false));

        oi.operatorController().getButton(OperatorControls.RunSpeakerShooterButton).whileTrue(new AutoShootCommand(shooter, shooterPivot, hopper));
        oi.operatorController().getButton(OperatorControls.RunAmpShooterButton).whileTrue(new AutoAmpCommand(shooter, shooterPivot, hopper));
        oi.operatorController().getButton(OperatorControls.ManualShooterButton).whileTrue(new ManualShootCommand(shooter, shooterPivot, hopper));

        oi.operatorController().getButton(OperatorControls.ShooterPivotTop).whileTrue(new RunCommand(() -> {
            shooterPivot.setAngleDegrees(ShooterInfo.SHOOTER_PIVOT_AMP_SETPOINT);
        }, shooterPivot));
        oi.operatorController().getButton(OperatorControls.ShooterPivotBot).whileTrue(new RunCommand(() -> {
            shooterPivot.setAngleDegrees(ShooterInfo.SHOOTER_PIVOT_BOTTOM_SETPOINT);
        }, shooterPivot));
    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }
}
