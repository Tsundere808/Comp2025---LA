package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.LED;


public class IntakeCommand extends Command{

    private final CoralIntakeSubsystem intake;
    
    private boolean firstcheck = true;

    private final LED led;

    public IntakeCommand(CoralIntakeSubsystem intake,LED led) {
        this.intake = intake;
        this.led = led;
        addRequirements(intake,led);

      }

      @Override
  public void initialize() {
    led.RED();
    intake.intake();  
  }

  @Override
  public void execute()
  {
    intake.intake();  
  }

      @Override
      public boolean isFinished() {
          return intake.coralCheck();
      }

     @Override
     public void end(boolean interrupted) {
      led.GREEN();
      intake.disable();
    }
}