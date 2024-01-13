// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IShooterSubsystem;
import frc.robot.subsystems.TwoWheelShooter;


public class RobotContainer
{
    private final IShooterSubsystem shooter;

    public RobotContainer()
    {
        shooter = new TwoWheelShooter(0, 1);
        configureBindings();
    }
    
    
    private void configureBindings() {
        shooter.setDefaultCommand(new Command() {
            {
                addRequirements(shooter);
            }

            @Override
            public void execute() {
                shooter.shoot(0.2);
            }
        });
    }
    
    
    public Command getAutonomousCommand()
    {
        return Commands.print("No autonomous command configured");
    }
}
