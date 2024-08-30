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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.AutonAimCommand;
import frc.robot.commands.shooter.*;
import frc.robot.constants.Controls.DriverControls;
import frc.robot.constants.Controls.OperatorControls;
import frc.robot.commands.climber.ExtendClimberCommand;
import frc.robot.commands.climber.ManualClimbCommand;
import frc.robot.commands.climber.RetractClimberCommand;
import frc.robot.commands.drive.AmpAimCommand;
import frc.robot.commands.drive.TeleopSwerveDriveCommand;
import frc.robot.commands.intake.*;
import frc.robot.constants.RobotInfo;
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
    public final SendableChooser<Integer> autonDelay;
    public static SendableChooser<Boolean> alliance;
    

    public RobotContainer() {
        oi = new OI();
        ledStrip = SubsystemManager.getLedStrip();

        swerveDriveSubsystem = SubsystemManager.getSwerveDrive();

        shooter = SubsystemManager.getShooter();
        intake = SubsystemManager.getIntake();
        hopper = SubsystemManager.getHopper();
        climber = SubsystemManager.getClimber();

        nameCommands();

        autonDelay = new SendableChooser<>();
        autonDelay.setDefaultOption("No Delay", 0);
        autonDelay.addOption("4s", 4);
        autonDelay.addOption("5s", 5);
        autonDelay.addOption("6s", 6);
        autonDelay.addOption("7s", 7);
        autonDelay.addOption("8s", 8);  
        autonDelay.addOption("9s", 9);
        autonDelay.addOption("10s", 10);
        autonDelay.addOption("11s", 11);

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
        NamedCommands.registerCommand("ManualSwerveDriveCommand", new TeleopSwerveDriveCommand(swerveDriveSubsystem, oi));
        NamedCommands.registerCommand("AutonAimCommand", new AutonAimCommand(swerveDriveSubsystem, RobotInfo.AimInfo.AIM_TIME));
        NamedCommands.registerCommand("SpinupCommand", new ShooterSpinupCommand(shooter));
        //intake commands
        NamedCommands.registerCommand("IntakeCommand", Commands.deadline(new WaitCommand(3), new IntakeCommand(intake, hopper, ledStrip)));
        NamedCommands.registerCommand("OuttakeCommand", new OuttakeCommand(intake, hopper));
        NamedCommands.registerCommand("ExtendIntake", new ExtendIntakeCommand(intake));
        NamedCommands.registerCommand("RetractIntake", new RetractIntakeCommand(intake));

        // shooting commands
        NamedCommands.registerCommand("ShootSpeaker", Commands.deadline(new WaitCommand(3), new AutoSpeakerCommand(shooter, hopper, ledStrip)));
        NamedCommands.registerCommand("ShootAmp", new AutoAmpCommand(shooter, hopper, ledStrip));
        NamedCommands.registerCommand("ManualSpeaker", new ManualShootCommand(shooter, hopper, ledStrip));
    }


    private void configureBindings() {
        Trigger inRange = new Trigger(AimUtil::inRange);

        swerveDriveSubsystem.setDefaultCommand(new TeleopSwerveDriveCommand(
                swerveDriveSubsystem, oi
        ));

        // TODO: extract to named class
        ledStrip.setDefaultCommand(new RunCommand(() -> {
            if(hopper.hasNote()) {
                ledStrip.usePattern(new PhasingLEDPattern(new Color8Bit(255, 50, 0), 3));
            }
            else {
                ledStrip.usePattern(new SolidLEDPattern(new Color8Bit(0, 0, 255)));
            }
        }, ledStrip));

        oi.driverController().getButton(DriverControls.ClimberExtendButton).whileTrue(new ExtendClimberCommand(climber));
        oi.driverController().getButton(DriverControls.ClimberRetractButton).whileTrue(new RetractClimberCommand(climber));
        oi.driverController().getButton(DriverControls.ClimberSwap1Button).whileTrue(new ManualClimbCommand(climber, 1, -1));
        oi.driverController().getButton(DriverControls.ClimberSwap2Button).whileTrue(new ManualClimbCommand(climber, -1, 1));

        oi.operatorController().getButton(OperatorControls.IntakeButton).whileTrue(new IntakeCommand(intake, hopper, ledStrip));
 
        oi.operatorController().getButton(OperatorControls.OuttakeButton).whileTrue(new OuttakeCommand(intake, hopper));

        oi.operatorController().getButton(OperatorControls.IntakeExtendButton).whileTrue(new ExtendIntakeCommand(intake));

        oi.operatorController().getButton(OperatorControls.IntakeRetractButton).whileTrue(new RetractIntakeCommand(intake));

        oi.operatorController().getButton(OperatorControls.RunSpeakerShooterButton).whileTrue(new AutoSpeakerCommand(shooter, hopper, ledStrip));
        oi.operatorController().getButton(OperatorControls.RunAmpShooterButton).whileTrue(new AutoAmpCommand(shooter, hopper, ledStrip));
        oi.operatorController().getButton(OperatorControls.ManualShooterButton).whileTrue(new ManualShootCommand(shooter, hopper, ledStrip));
        oi.operatorController().getButton(OperatorControls.RunTrapShooterButton).whileTrue(new AutoTrapCommand(shooter, hopper, ledStrip));
        oi.operatorController().getButton(OperatorControls.LaunchShooterButton).whileTrue(new LaunchCommand(shooter, hopper, ledStrip));
        oi.operatorController().getButton(OperatorControls.ShooterIntake).whileTrue(new ShooterIntakeCommand(shooter, hopper, ledStrip));

        oi.operatorController().getButton(OperatorControls.ToggleIR).onTrue(hopper.toggleIR());

        oi.driverController().getButton(DriverControls.AmpAlignButton).whileTrue(new AmpAimCommand(swerveDriveSubsystem));

        oi.driverController().getButton(DriverControls.ResetGyroButton1).and(
                oi.driverController().getButton(DriverControls.ResetGyroButton2)
        ).whileTrue(new RunCommand(swerveDriveSubsystem::reset));

        inRange.whileTrue(new ShooterSpinupCommand(shooter));
    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }
}
