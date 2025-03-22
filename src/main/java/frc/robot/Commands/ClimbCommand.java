package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class ClimbCommand extends Command{

    private final Climber climber;
    private final LED led;    
    private boolean firstcheck = true;
    private final ShoulderSubsystem shoulder;
    private final PivotSubsystem pivot;

    public ClimbCommand(Climber climber, LED led,ShoulderSubsystem shoulder, PivotSubsystem pivot) {
        this.climber = climber;
        this.led = led;
        this.pivot = pivot;
        this.shoulder = shoulder;
        
        addRequirements(climber,led,pivot,shoulder);

      }

      @Override
  public void initialize() {
    climber.slowDown();
    shoulder.withPosition(47.78);
    pivot.withPosition(0);
  }

  @Override
  public void execute()
  {
    climber.slowDown();
  }

      @Override
      public boolean isFinished() {
          return !climber.heightCheck();
      }

     @Override
     public void end(boolean interrupted) {
      climber.setHoldPosition(climber.getEncoder());
      led.DGREEN();
      
    }
}