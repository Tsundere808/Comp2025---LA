// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class ElevatorHomeCommand extends Command {
  private final ElevatorSubsystem elevator; 
  private final LED led;

  /** Creates a new AmpShot. */
  public ElevatorHomeCommand(ElevatorSubsystem elevator, LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.led = led;
    addRequirements(elevator);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   elevator.setPosition(0);
   led.PINK();
  }

  @Override
  public boolean isFinished() {
    return elevator.CheckPositionHome();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.DGREEN();
  }


}
