package frc.robot.subsystems;

//Import required WPILib libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class LED extends SubsystemBase{

        Spark blinkin;

        public LED() {
            blinkin = new Spark(9);
        }
        public void RED() {
            blinkin.set(0.61);
        }
        public void PINK() {
          blinkin.set(0.57);
      }
      public void DGREEN() {
        blinkin.set(0.75);
    }
    public void HBFAST() {
      blinkin.set(0.27);
  }
        
        /**
         * if the robot detects the cube, the LED blinks gold (-0.07)
         */
        public void GREEN() {
            blinkin.set(0.77); 
        }

        public  Command setRED()
        {
            return runOnce(() -> this.RED());        }

        public Command setGREEN()
        {
            return runOnce(() -> this.GREEN());
        }
        public  Command setPINK()
        {
            return runOnce(() -> this.PINK());        }

        public Command setHBFAST()
        {
            return runOnce(() -> this.HBFAST());
        }
        public Command setDGREEN()
        {
            return runOnce(() -> this.DGREEN());
        }

}
