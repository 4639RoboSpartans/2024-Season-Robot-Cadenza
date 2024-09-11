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
                        swerveDriveSubsystem.pathfindCommand(
                                new Pose2d(AimUtil.getAmpPose(), AimUtil.getAmpRotation())
                        )
                );

        DriverControls.AimButton
                .whileTrue(
                        swerveDriveSubsystem.pathfindCommand(
                                AimUtil.getManualSpeakerPose()
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

        OperatorControls.IntakeButton.whileTrue(new IntakeCommand(intake, hopper, ledStrip, oi));

        OperatorControls.OuttakeButton.whileTrue(new OuttakeCommand(intake, hopper));

        OperatorControls.IntakeExtendButton.whileTrue(new ExtendIntakeCommand(intake));

        OperatorControls.IntakeRetractButton.whileTrue(new RetractIntakeCommand(intake));

        OperatorControls.RunSpeakerShooterButton.whileTrue(new AutoSpeakerCommand(shooter, hopper, ledStrip));
        OperatorControls.RunAmpShooterButton.whileTrue(new AutoAmpCommand(intake));
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

        OperatorControls.resetIntakeOffset2.and(OperatorControls.resetIntakeOffset2).whileTrue(
                new ManualIntakeExtendCommand(intake)
        );
    }

    public Command getDelay() {
        return Commands.waitSeconds(delayTime.getSelected());
    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }
}

/*
 * :30:24.782 PM
 	at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:365)  	
    at edu.wpi.first.wpilibj.TimedRobot.startCompetition(TimedRobot.java:131)  
    	at edu.wpi.first.wpilibj.IterativeRobotBase.loopFunc(IterativeRobotBase.java:345)  
        	at frc.robot.Robot.autonomousInit(Robot.java:89)  	
            at frc.robot.RobotContainer.getAutonomousCommand(RobotContainer.java:174) 
             	at edu.wpi.first.wpilibj2.command.Commands.sequence(Commands.java:192)
                 ERROR  1  The startCompetition() method (or methods called by it) should have handled the exception above. 
                 
    edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:386)  	
    at edu.wpi.first.wpilibj2.command.SequentialCommandGroup.<init>(SequentialCommandGroup.java:33) 
     	at edu.wpi.first.wpilibj2.command.SequentialCommandGroup.addCommands(SequentialCommandGroup.java:47)  
        	at edu.wpi.first.wpilibj2.command.CommandScheduler.registerComposedCommands(CommandScheduler.java:599)
             Warning  1  The robot program quit unexpectedly. This is usually due to a code error.
  The above stacktrace can help determine where the error occurred.
  See https://wpilib.org/stacktrace for more information.  edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:379)  
  
  Error at frc.robot.RobotContainer.getAutonomousCommand(RobotContainer.java:174): Unhandled exception: java.lang.Exception
  : Originally composed at: ERROR
    1  Unhandled exception: java.lang.Exception: Originally composed at:  frc.robot.RobotContainer.getAutonomousCommand(RobotContainer.java:174) 

 */