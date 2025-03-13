// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class IntakePositionCommand extends Command {
  private final PivotSubsystem pivot;
  private final ElevatorSubsystem elevator; 
  private final ShoulderSubsystem shoulder;

  /** Creates a new AmpShot. */
  public IntakePositionCommand(PivotSubsystem pivot,ElevatorSubsystem elevator,ShoulderSubsystem shoulder) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.elevator = elevator;
    this.shoulder = shoulder;
    addRequirements(pivot,elevator,shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   // pivot.ampPositionCommand();
   // elevator.ampPosition();
  }

  @Override
  public boolean isFinished() {
    return elevator.CheckPositionL4() && pivot.CheckPositionL4() && shoulder.CheckPositionL4();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }


}
