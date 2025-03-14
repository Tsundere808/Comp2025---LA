// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//Test Commit

import static edu.wpi.first.units.Units.*;

import java.util.EventListener;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.ElevatorHomeCommand;
import frc.robot.Commands.ElevatorL4Command;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.OuttakeCommand;
import frc.robot.Commands.SWL4Command;
import frc.robot.Commands.SWLoadCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.Constants.CoralIntakeConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralIntakeSubsystem;
//import frc.robot.subsystems.DeAlgae;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //Elevator Manual works
    private ElevatorSubsystem elevator = new ElevatorSubsystem();

    private PivotSubsystem pivot = new PivotSubsystem();

    private ShoulderSubsystem shoulderSubsystem= new ShoulderSubsystem();
    private LED led = new LED();

    //Climber Manual works
    private Climber climber = new Climber();

    //private DeAlgae deAlgae = new DeAlgae();

    private CoralIntakeSubsystem coralintake = new CoralIntakeSubsystem();

    private AlgaeSubsystem algaeSubsystem = new AlgaeSubsystem();
    //private ClimberNEW climberNEW = new ClimberNEW();
    
    private IntakeCommand intakeCommand = new IntakeCommand(coralintake, led);
    private OuttakeCommand outtakeCommand = new OuttakeCommand(coralintake, led);

    private ElevatorL4Command elevatorL4Command = new ElevatorL4Command(elevator, led);
    private ElevatorHomeCommand elevatorHomeCommand = new ElevatorHomeCommand(elevator, led);
    private SWL4Command swL4Command = new SWL4Command(pivot, shoulderSubsystem,led);
    private SWLoadCommand swLoadCommand = new SWLoadCommand(pivot, shoulderSubsystem, led);

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandJoystick opJoystick = new CommandJoystick(1);
    private final CommandJoystick opJoystick2 = new CommandJoystick(2);

    private final SendableChooser<Command> autoChooser;

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {

        

        //Named Commands
        //Drive
        NamedCommands.registerCommand("setFieldRelative", drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Elevator
        NamedCommands.registerCommand("elevatorL4", elevatorL4Command);
        NamedCommands.registerCommand("elevatorHome", elevatorHomeCommand);

        //SW
        NamedCommands.registerCommand("swL4", swL4Command);
        NamedCommands.registerCommand("swLoad", swLoadCommand);

        //Intake
        NamedCommands.registerCommand("intake", intakeCommand);
        NamedCommands.registerCommand("outtake", outtakeCommand);

        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
   
        //Operator Controls
        //L4 + L3
        //opJoystick2.button(5).onTrue(new ParallelCommandGroup(pivot.withPosition(-33.9), shoulderSubsystem.setL4Position())); //34.7
        //Load
        //opJoystick2.button(1).onTrue(new ParallelCommandGroup(pivot.withPosition(26.18), shoulderSubsystem.withPosition(11.91)));
        //Home
        opJoystick2.button(7).onTrue(new ParallelCommandGroup(pivot.withPosition(0), shoulderSubsystem.setHomePosition()));

        opJoystick2.button(6).whileTrue(shoulderSubsystem.slowUp());
        opJoystick2.button(4).whileTrue(shoulderSubsystem.slowDown());

        //L2
        //opJoystick2.button(3).onTrue(new ParallelCommandGroup(pivot.withPosition(15.4), shoulderSubsystem.withPosition(11.91)));

        //L1
        //opJoystick2.button(4).onTrue(new ParallelCommandGroup(pivot.withPosition(15.4), shoulderSubsystem.withPosition(11.91)));

        //L4 Elevator
        opJoystick2.button(12).onTrue(elevator.setL4Position());
        opJoystick2.button(11).onTrue(elevator.setHomePosition());

        //Elevator Code

        opJoystick2.button(10).whileTrue(elevator.withPosition(22.5));
        // elevator.setDefaultCommand(elevator.stopCommand());
        // opJoystick.button(6).whileTrue(elevator.slowUp().andThen(elevator.stopCommand()));
        // opJoystick.button(4).whileTrue(elevator.slowDown().andThen(elevator.stopCommand()));
        // opJoystick.button(5).whileTrue(elevator.setL4Position());
        // opJoystick.button(3).whileTrue(elevator.setHomePosition());
        
        //Pivot Controls
         pivot.setDefaultCommand(pivot.stopCommand());
         opJoystick2.button(5).whileTrue(pivot.slowUp());
         opJoystick2.button(3).whileTrue(pivot.slowDown());
         //opJoystick2.button(1).whileTrue(pivot.withPosition(26.18));
         //opJoystick2.button(2).whileTrue(pivot.withPosition(-34.7));
         //opJoystick2.button(7).whileTrue(pivot.withPosition(0));

         //Algae
         algaeSubsystem.setDefaultCommand(algaeSubsystem.stop());
         opJoystick2.button(9).whileTrue(new ParallelCommandGroup (algaeSubsystem.withVelocity(60), shoulderSubsystem.withPosition(47.78), pivot.withPosition(0)));
         opJoystick2.button(10).whileTrue(new ParallelCommandGroup (algaeSubsystem.withVelocity(60), shoulderSubsystem.withPosition(45.78), pivot.withPosition(0), elevator.withPosition(40)));

         //opJoystick.button(9).whileTrue(algaeSubsystem.withVelocity(-70));


        //Shoulder Controls
        shoulderSubsystem.setDefaultCommand(shoulderSubsystem.stopCommand());
        

        //L4 + L3
        joystick.y().onTrue(new ParallelCommandGroup(pivot.withPosition(-33.9), shoulderSubsystem.setL4Position()));
        //L2
        joystick.a().onTrue(new ParallelCommandGroup(pivot.withPosition(15.4), shoulderSubsystem.withPosition(11.91)));
        //Load
        joystick.rightBumper().onTrue(new ParallelCommandGroup( pivot.withPosition(26.18), shoulderSubsystem.withPosition(11.91)));
        joystick.rightTrigger().onTrue(new ParallelCommandGroup( pivot.withPosition(4.87), shoulderSubsystem.withPosition(34.4)));
        joystick.rightBumper().onTrue(intakeCommand);
        joystick.rightTrigger().onTrue(intakeCommand);

        
        //joystick.rightTrigger().onTrue(new ParallelCommandGroup(pivot.withPosition(26.18), shoulderSubsystem.withPosition(11.91)));

        

        // joystick.pov(90).whileTrue(shoulderSubsystem.setL4Position()); 
        // joystick.pov(270).whileTrue(shoulderSubsystem.setHomePosition());
        // joystick.rightTrigger().whileTrue(shoulderSubsystem.withPosition(11.91));
//43.78

//////////////
        //Climber Code
        climber.setDefaultCommand(climber.stopCommand());
        opJoystick2.button(1).whileTrue(climber.slowDown());
        opJoystick2.button(2).whileTrue(climber.slowUp());

        opJoystick2.pov(0).onTrue(climber.withPosition(270)); // For the match
        opJoystick2.pov(90).onTrue(climber.withPosition(0)); // For the match
        opJoystick2.pov(180).onTrue(climber.withPosition(-120)); //Climb -100 from center
        //opJoystick2.pov(0).whileTrue(climber.withPosition(-270)); // For the pid  

///////////////

        //Coral Controls
        coralintake.setDefaultCommand(coralintake.stop());
        // joystick.leftBumper().whileTrue(coralintake.intakeCommand());
        joystick.leftTrigger().whileTrue(new ParallelCommandGroup(coralintake.outtakeCommand(), led.setRED()));


       
        //DeAlgae Controls
        // joystick.leftBumper().whileTrue(deAlgae.intakeCommand());
        // joystick.leftTrigger().whileTrue(deAlgae.outtakeCommand());


        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * .6) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * .6) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        //             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        //             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        //Drivetrain Controls
        
       
        // joystick.start().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));


        joystick.back().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.pov(0).whileTrue(new ParallelCommandGroup (drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.25).withVelocityY(0))
        , led.setHBFAST()));

        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.25).withVelocityY(0))
        );

        joystick.pov(90).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.25))
        );
        joystick.pov(270).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.25))
        );

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // drivetrain.registerTelemetry(logger::telemeterize);

        joystick.x().whileTrue(drivetrain.resetPIDsReef().andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(drivetrain.run_xControllerLeft())
            .withVelocityY(drivetrain.run_yControllerLeft())
            .withRotationalRate(drivetrain.run_omegaControllerLeft())
            )));

        joystick.b().whileTrue(drivetrain.resetPIDsReef().andThen(drivetrain.applyRequest(() ->
        drive.withVelocityX(drivetrain.run_xControllerRight())
            .withVelocityY(drivetrain.run_yControllerRight())
            .withRotationalRate(drivetrain.run_omegaControllerRight())
            )));

        joystick.start().onTrue(drivetrain.poseResetCommandReef());

        ///////////////////////////////////////////////////////////////////////////////////////

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
