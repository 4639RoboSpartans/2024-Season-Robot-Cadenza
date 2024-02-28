// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Controls.DriverControls;
import frc.robot.Constants.Controls.OperatorControls;
import frc.robot.commands.shooter.ManualShootCommand;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.ManualClimbCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.drive.ManualSwerveDriveCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.OuttakeCommand;
import frc.robot.commands.intake.SetIntakeExtendedCommand;
import frc.robot.commands.intake.ToggleIRCommand;
import frc.robot.commands.shooter.AutoAmpCommand;
import frc.robot.commands.shooter.AutoShootCommand;
import frc.robot.oi.OI;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SubsystemCreator;
import frc.robot.subsystems.climber.IClimberSubsystem;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.sensors.IRSensor;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.swerve.AimSubsystem;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;

@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class RobotContainer {
    private final OI oi;
    private final NavX navX;

    private final ISwerveDriveSubsystem swerveDriveSubsystem;

    private final IShooterSubsystem shooter;
    private final IIntakeSubsystem intake;
    private final IClimberSubsystem climber;
    private final AimSubsystem aimSubsystem;
    private final IHopperSubsystem hopper;

    private final IRSensor ir;
    private SendableChooser<Command> autos;

    public RobotContainer() {
        oi = new OI();
        navX = new NavX();
        aimSubsystem = new AimSubsystem();
        ir = new IRSensor();

        swerveDriveSubsystem = SubsystemCreator.getSwerveDrive();

        shooter = SubsystemCreator.getShooter();
        intake = SubsystemCreator.getIntake();
        hopper = SubsystemCreator.getHopper();
        climber = SubsystemCreator.getClimber();

        autos = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autons", autos);

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
        NamedCommands.registerCommand("AutoShootCommand", new AutoShootCommand(shooter, hopper));



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

        oi.operatorController().getButton(OperatorControls.RunSpeakerShooterButton).whileTrue(new AutoShootCommand(shooter, hopper));
        oi.operatorController().getButton(OperatorControls.RunAmpShooterButton).whileTrue(new AutoAmpCommand(shooter, hopper));
        oi.operatorController().getButton(OperatorControls.ManualShooterButton).whileTrue(new ManualShootCommand(shooter, hopper));

        oi.operatorController().getButton(OperatorControls.ToggleIR).whileTrue(new ToggleIRCommand(ir));
    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }
}
