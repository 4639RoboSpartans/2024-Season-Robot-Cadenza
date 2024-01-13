// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.IShooterSubsystem;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.shooter.TwoWheelShooterNeos;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;


public class RobotContainer
{
    private final IShooterSubsystem shooter;

    private final SwerveDriveSubsystem swerveDriveSubsystem;

    private final NavX navX;

    public RobotContainer()
    {
        shooter = new TwoWheelShooterNeos(13, 14);
        navX = new NavX();
        swerveDriveSubsystem = new SwerveDriveSubsystem(navX);
        configureBindings();
    }
    
    
    private void configureBindings() {
//        shooter.setDefaultCommand(new Command() {
//            {
//                addRequirements(shooter);
//            }
//
//            @Override
//            public void execute() {
//                shooter.shoot(0.2);
//            }
//        });
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
