// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.Controls.DriverControls;
import frc.robot.constants.InterpolatingTables;
import frc.robot.oi.OI;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.robot.subsystems.shooter.shooter.Shooter;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.ISwerveDriveSubsystem;
import frc.robot.util.AimUtil;
import frc.robot.util.AutoHelper;

@SuppressWarnings({"FieldCanBeLocal", "unused"})
public class RobotContainer {
    public static OI oi;
    private final ISwerveDriveSubsystem swerveDriveSubsystem;
    private final SendableChooser<Command> autos;
    private final SendableChooser<Double> delayTime;
    public static SendableChooser<Boolean> alliance;


    public RobotContainer() {
        InterpolatingTables.initializeTables();

        swerveDriveSubsystem = CommandSwerveDrivetrain.getInstance();

        autos = new SendableChooser<>();
        autos.setDefaultOption("null auto", new WaitCommand(1));
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

    private void configureBindings() {
        swerveDriveSubsystem.setDefaultCommand(
                swerveDriveSubsystem.driveFieldCentricCommand()
        );

        DriverControls.SOTF
                .whileTrue(
                        swerveDriveSubsystem.SOTFCommand()
                );

        DriverControls.AimButton
                .whileTrue(
                        Commands.parallel(
                                swerveDriveSubsystem.pathfindCommand(
                                        AimUtil.getManualSpeakerPose()
                                )
                        )
                );
    }

    public Command getDelay() {
        return Commands.waitSeconds(delayTime.getSelected());
    }

    public Command getAutonomousCommand() {
        return autos.getSelected();
    }

    public void sendSubsystems() {
        SmartDashboard.putData(swerveDriveSubsystem);
        SmartDashboard.putData(Intake.getInstance());
        SmartDashboard.putData(Hopper.getInstance());
        SmartDashboard.putData(Shooter.getInstance());
        SmartDashboard.putData(Pivot.getInstance());
    }
}