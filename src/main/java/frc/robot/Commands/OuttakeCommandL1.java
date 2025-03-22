package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.LED;
import edu.wpi.first.wpilibj.Timer;


public class OuttakeCommandL1 extends Command{

    private final CoralIntakeSubsystem intake;
    private final LED led;
    private Timer timer = new Timer();
    
    private boolean firstcheck = true;

    public OuttakeCommandL1(CoralIntakeSubsystem intake, LED led) {
        this.intake = intake;
        this.led = led;
        addRequirements(intake);

      }

      @Override
  public void initialize() {
    intake.outtakeAUTO();  
    timer.restart();
    timer.start();
  }

  @Override
  public void execute()
  {
    intake.outtakeAUTO();  
  }

      @Override
      public boolean isFinished() {
          return !intake.coralCheck() && timer.get() > 1;
      }

     @Override
     public void end(boolean interrupted) {
      intake.disable();
      led.RED();
      
    }
}