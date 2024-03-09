// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.constants.Constants.Controls.DriverControls;
import frc.robot.constants.Constants.Controls.OperatorControls;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.ManualClimbCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.drive.AmpAimCommand;
import frc.robot.commands.drive.TeleopSwerveDriveCommand;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.AutoAmpCommand;
import frc.robot.commands.shooter.AutoShootCommand;
import frc.robot.commands.shooter.AutoTrapCommand;
import frc.robot.commands.shooter.ManualShootCommand;
import frc.robot.led.LEDStrip;
import frc.robot.led.SolidLEDPattern;
import frc.robot.oi.OI;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.aim.AimSubsystem;
import frc.robot.subsystems.climber.IClimberSubsystem;
import frc.robot.subsystems.hopper.IHopperSubsystem;
import frc.robot.subsystems.intake.IIntakeSubsystem;
import frc.robot.subsystems.sensors.IRSensor;
import frc.robot.subsystems.shooter.IShooterSubsystem;
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
    private final IRSensor ir2;
    private final LEDStrip ledStrip;

    private final SendableChooser<Command> autos;
    public static SendableChooser<Boolean> alliance;

    public RobotContainer() {
        oi = new OI();
        navX = SubsystemManager.getNavX();
        aimSubsystem = SubsystemManager.getAimSubsystem();
        ir = SubsystemManager.getIRSensor();
        ir2 = SubsystemManager.getIrSensor2();
        ledStrip = SubsystemManager.getLedStrip();

        swerveDriveSubsystem = SubsystemManager.getSwerveDrive();

        shooter = SubsystemManager.getShooter();
        intake = SubsystemManager.getIntake();
        hopper = SubsystemManager.getHopper();
        climber = SubsystemManager.getClimber();

        nameCommands();

        autos = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autons", autos);

        alliance = new SendableChooser<>();
        alliance.addOption("Red", true);
        alliance.setDefaultOption("Blue", false);
        SmartDashboard.putData("Alliance", alliance);

        configureBindings();
    }

    private void nameCommands(){
        //climber commands
        NamedCommands.registerCommand("ExtendClimberCommand", new ExtendClimberCommand(climber));
        NamedCommands.registerCommand("ManualClimbCommand", new ManualClimbCommand(climber, 0, 0));
        NamedCommands.registerCommand("RetractClimberCommand", new RetractClimberCommand(climber));
        //drive commands
        NamedCommands.registerCommand("ManualSwerveDriveCommand", new TeleopSwerveDriveCommand(swerveDriveSubsystem, aimSubsystem, oi));
        //intake commands
        NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(intake, hopper));
        NamedCommands.registerCommand("OuttakeCommand", new OuttakeCommand(intake, hopper));
        NamedCommands.registerCommand("ExtendIntake", new ExtendIntakeCommand(intake));
        NamedCommands.registerCommand("RetractIntake", new RetractIntakeCommand(intake));

        // shooting commands
        NamedCommands.registerCommand("ShootSpeaker", new AutoShootCommand(shooter, hopper, ledStrip));
        NamedCommands.registerCommand("ShootAmp", new AutoAmpCommand(shooter, hopper, ledStrip));
        NamedCommands.registerCommand("ManualSpeaker", new ManualShootCommand(shooter, hopper, ledStrip));
    }


    private void configureBindings() {
        swerveDriveSubsystem.setDefaultCommand(new TeleopSwerveDriveCommand(
                swerveDriveSubsystem, aimSubsystem, oi
        ));

        // TODO: extract to named class
        ledStrip.setDefaultCommand(new RunCommand(() -> {
            if(ir.hasNote()) {
                ledStrip.usePattern(new SolidLEDPattern(new Color8Bit(255, 50, 0)));
            }
            else {
                ledStrip.resetToBlank();
            }
        }, ledStrip));

        oi.driverController().getButton(DriverControls.ClimberExtendButton).whileTrue(new ExtendClimberCommand(climber));
        oi.driverController().getButton(DriverControls.ClimberRetractButton).whileTrue(new RetractClimberCommand(climber));
        oi.driverController().getButton(DriverControls.ClimberSwap1Button).whileTrue(new ManualClimbCommand(climber, 1, -1));
        oi.driverController().getButton(DriverControls.ClimberSwap2Button).whileTrue(new ManualClimbCommand(climber, -1, 1));

        oi.operatorController().getButton(OperatorControls.IntakeButton).whileTrue(new IntakeCommand(intake, hopper));
 
        oi.operatorController().getButton(OperatorControls.OuttakeButton).whileTrue(new OuttakeCommand(intake, hopper));

        oi.operatorController().getButton(OperatorControls.IntakeExtendButton).whileTrue(new ExtendIntakeCommand(intake));

        oi.operatorController().getButton(OperatorControls.IntakeRetractButton).whileTrue(new RetractIntakeCommand(intake));

        oi.operatorController().getButton(OperatorControls.RunSpeakerShooterButton).whileTrue(new AutoShootCommand(shooter, hopper, ledStrip));
        oi.operatorController().getButton(OperatorControls.RunAmpShooterButton).whileTrue(new AutoAmpCommand(shooter, hopper, ledStrip));
        oi.operatorController().getButton(OperatorControls.ManualShooterButton).whileTrue(new ManualShootCommand(shooter, hopper, ledStrip));
        oi.operatorController().getButton(OperatorControls.RunTrapShooterButton).whileTrue(new AutoTrapCommand(shooter, hopper, ledStrip));

        oi.operatorController().getButton(OperatorControls.ToggleIR).whileTrue(new ToggleIRCommand(ir));

        oi.driverController().getButton(DriverControls.AmpAlignButton).whileTrue(new AmpAimCommand(swerveDriveSubsystem, aimSubsystem));
    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }
}
