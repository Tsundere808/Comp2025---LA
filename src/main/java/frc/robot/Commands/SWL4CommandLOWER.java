// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class SWL4CommandLOWER extends Command {
  private final PivotSubsystem pivot;
  private final ShoulderSubsystem shoulder;
  private final LED led;

  /** Creates a new AmpShot. */
  public SWL4CommandLOWER(PivotSubsystem pivot,ShoulderSubsystem shoulder,LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pivot = pivot;
    this.shoulder = shoulder;
    this.led = led;
    addRequirements(pivot,shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pivot.setPosition(-34.9);
    shoulder.setPosition(88.3);
    led.PINK();
  }

  @Override
  public boolean isFinished() {
    return pivot.CheckPositionL4LOWER() && shoulder.CheckPositionL4();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.DGREEN();
  }


}
