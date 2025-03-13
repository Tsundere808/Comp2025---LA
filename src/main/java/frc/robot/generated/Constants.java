// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.generated;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static final class CoralIntakeConstants{
    public static final int intakeID = 31;
    public static final int intakeSpeed = -20;
    public static final int outtakeSpeed = 20;

  }
  public static final class ShooterConstants {
    public static final int topShooter = 34;
    public static final int botShooter = 33;
    public static final int shootSpeed = 10;
}
public static final class FeederConstants {
  public static final int feederID = 32;
  public static final int feederEncoderA = 5;
  public static final int feederEncoderB = 6;
  public static final double IntakeSPEED = .2;
  public static final double OutakeSPEED = -.2;
  public static final double ampOutakeSPEED = 60;
}
public static final class PivotConstants {
        
  public static final int pivot = 40;
  public static final int pivotspeed = 20;
  public static final double homePosition = 0.1;
  public static final int intakePosition = -10;
  public static final int subwooferShotPosition = -19;
  public static final int farShot = -29;
  public static final int extremeFarShot = -10;
  public static final int elevatorSubwooferShotPosition = -10;
  public static final int elevatorFarShot = -10;
  public static final double PIVOTMAX = -18.5;

}
public static final class ElevatorConstants {
        
  public static final int leftElevator = 14;
  public static final double eHomePos = 0;
  public static final double eAmp = -50;
  public static final double efarshot = -50;
  public static final double eextremefarshot = -50;
  public static final double eClimbPos = -25;
  public static final double ELEVATORMAX = -50.21;

}
public static final class ClimberConstants
{
  public static final int climber = 41;
  public static final int cHomePos = 0;
  public static final int cUpPose = 61;
  public static final int cClimbPos = 100; 
  
}

public static final class AprilTagConstants  {

    //Reef April Tags Blue
    //April Tag 17
    public static final Pose3d aprilTag17_Position = new Pose3d(
        Units.inchesToMeters(160.39), 
        Units.inchesToMeters(130.17), 
        Units.inchesToMeters(12.13),
        new Rotation3d(0, 0, Units.degreesToRadians(240))
    );

    //April Tag 18
    public static final Pose3d aprilTag18_Position = new Pose3d(
        Units.inchesToMeters(144.00), //3.6576 m
        Units.inchesToMeters(158.50), //4.0259 m
        Units.inchesToMeters(12.13),
        new Rotation3d(0, 0, Units.degreesToRadians(180))
    );

    //April Tag 19
    public static final Pose3d aprilTag19_Position = new Pose3d(
        Units.inchesToMeters(160.39), 
        Units.inchesToMeters(186.83), 
        Units.inchesToMeters(12.13),
        new Rotation3d(0, 0, Units.degreesToRadians(120))
    );
    
    //April Tag 20
    public static final Pose3d aprilTag20_Position = new Pose3d(
        Units.inchesToMeters(193.10), 
        Units.inchesToMeters(186.83), 
        Units.inchesToMeters(12.13),
        new Rotation3d(0, 0, Units.degreesToRadians(60))
    );

    //April Tag 21
    public static final Pose3d aprilTag21_Position = new Pose3d(
        Units.inchesToMeters(209.49), 
        Units.inchesToMeters(158.50),
        Units.inchesToMeters(12.13), 
        new Rotation3d(0, 0, Units.degreesToRadians(0))
    );

    //April Tag 22
    public static final Pose3d aprilTag22_Position = new Pose3d(
        Units.inchesToMeters(193.10), 
        Units.inchesToMeters(130.17), 
        Units.inchesToMeters(12.13),
        new Rotation3d(0, 0, Units.degreesToRadians(300))
    );

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    //Reef April Tags Red
    //April Tag 6
    public static final Pose3d aprilTag6_Position = new Pose3d(
        Units.inchesToMeters(530.49), 
        Units.inchesToMeters(130.17), 
        Units.inchesToMeters(12.13),
        new Rotation3d(0, 0, Units.degreesToRadians(300))
    );

    //April Tag 7
    public static final Pose3d aprilTag7_Position = new Pose3d(
        Units.inchesToMeters(546.87), 
        Units.inchesToMeters(158.50), 
        Units.inchesToMeters(12.13),
        new Rotation3d(0, 0, Units.degreesToRadians(0))
    );

    //April Tag 8
    public static final Pose3d aprilTag8_Position = new Pose3d(
        Units.inchesToMeters(530.49), 
        Units.inchesToMeters(186.83), 
        Units.inchesToMeters(12.13),
        new Rotation3d(0, 0, Units.degreesToRadians(60))
    );
    
    //April Tag 9
    public static final Pose3d aprilTag9_Position = new Pose3d(
        Units.inchesToMeters(497.77), 
        Units.inchesToMeters(186.83), 
        Units.inchesToMeters(12.13),
        new Rotation3d(0, 0, Units.degreesToRadians(120))
    );

    //April Tag 10
    public static final Pose3d aprilTag10_Position = new Pose3d(
        Units.inchesToMeters(481.39), 
        Units.inchesToMeters(158.50), 
        Units.inchesToMeters(12.13),
        new Rotation3d(0, 0, Units.degreesToRadians(180))
    );

    //April Tag 11
    public static final Pose3d aprilTag11_Position = new Pose3d(
        Units.inchesToMeters(497.77), 
        Units.inchesToMeters(130.17), 
        Units.inchesToMeters(12.13),
        new Rotation3d(0, 0, Units.degreesToRadians(240))
    );
}
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


public static final class autoAlignTransformConstants{

    //Reef Transforms
    public static final Transform3d reefGoalPoseTransform_Left = new Transform3d( 
        new Translation3d(0.45, -Units.inchesToMeters(6.75), 0), //6.75
        new Rotation3d(0, 0,  Math.toRadians(180))
    );
        
    public static final Transform3d reefGoalPoseTransform_Right = new Transform3d( 
        new Translation3d(0.45, Units.inchesToMeters(6.75), 0), //0.17145 
        new Rotation3d(0, 0,  Math.toRadians(180))
    );

    }
}
