package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.generated.Constants.AprilTagConstants;
import frc.robot.generated.Constants.autoAlignTransformConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;


/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */

    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();

        this.getPigeon2().reset();

        xController.setTolerance(Units.inchesToMeters(1.5));
        yController.setTolerance(Units.inchesToMeters(1.5));
        omegaController.setTolerance(Units.degreesToRadians(0.5));                                                    
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
        System.out.println("THIS ACTUALLY WORKS");

        }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }
    
    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants        Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency    The frequency to run the odometry loop. If
     *                                   unspecified or set to 0 Hz, this is 250 Hz on
     *                                   CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation  The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                    Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                      //Pose Estimation Reef//
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Pose Estimator
    SwerveDrivePoseEstimator poseEstimatorReef = new SwerveDrivePoseEstimator(
        this.getKinematics(), 
        this.getPigeon2().getRotation2d(), 
        new SwerveModulePosition[] {
            this.getModule(0).getPosition(true),
            this.getModule(1).getPosition(true),
            this.getModule(2).getPosition(true),
            this.getModule(3).getPosition(true)
        }, 
        new Pose2d(),
        VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(1)),
        VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(0.1))
    );

   
    //Updates the field relative position of the robot
    public void updateOdometryReef() {
      poseEstimatorReef.update(
        this.getPigeon2().getRotation2d(),
        new SwerveModulePosition[] {
          this.getModule(0).getPosition(true),
          this.getModule(1).getPosition(true),
          this.getModule(2).getPosition(true),
          this.getModule(3).getPosition(true)
      });
  
      boolean useMegaTag2 = true;
      boolean doRejectUpdate = false;

      try {
      if (useMegaTag2 == false) {
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-backup");
          if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1 && LimelightHelpers.getBotPose2d_wpiBlue( "limelight-backup") != null ) {
              if(mt1.rawFiducials[0].ambiguity > 0.7) {
                  doRejectUpdate = true;
              } 
              if (mt1.rawFiducials[0].distToCamera > 3) {
                  doRejectUpdate = true;
              }
          }
          if (mt1.tagCount == 0) {
              doRejectUpdate = true;
          }

          if (!doRejectUpdate) {
              poseEstimatorReef.setVisionMeasurementStdDevs(VecBuilder.fill(.03,.03, 0.05));
              poseEstimatorReef.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
          }     
      }

      else if (useMegaTag2 == true) {

        LimelightHelpers.SetRobotOrientation("limelight-backup", poseEstimatorReef.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-backup");
        if (Math.abs(this.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720 && LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-backup") != null) {
          doRejectUpdate = true;
        }
        if (mt2.tagCount == 0) {
          doRejectUpdate = true;
        }
        if (!doRejectUpdate) {
          poseEstimatorReef.setVisionMeasurementStdDevs(VecBuilder.fill(.03, .03, 0.05));
          poseEstimatorReef.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
        }
      }
    }
    catch (Exception e) {
      //System.out.println("There is nothing coming from limelight");
    }
  }
    
    //Gets the current position of the robot on the field
    public Pose2d getCurrentPoseReef() {
        return poseEstimatorReef.getEstimatedPosition();
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                             //Important Set Up For Driving To Tag//
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    //Profiled PID Stuff
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 2);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(4, 3);

    private static final Transform3d TAG_TO_GOAL = new Transform3d(
        new Translation3d(1, 0, 0),
        new Rotation3d(0, 0, Math.PI));

    private final ProfiledPIDController xController = new ProfiledPIDController(1.9, 0, 0, X_CONSTRAINTS); //1.9, 1
    private final ProfiledPIDController yController = new ProfiledPIDController(1.9, 0, 0, Y_CONSTRAINTS); //1.9, 1
    private final ProfiledPIDController omegaController = new ProfiledPIDController(2.6, 0, 0, OMEGA_CONSTRAINTS); //3.3, 1.4

    //Units
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    // Reef Reset PIDs
    public void resetPIDControllersReef() {
        var robotPose = this.getCurrentPoseReef();
        omegaController.reset(robotPose.getRotation().getRadians());
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
    }

    public Command resetPIDsReef () {
      return runOnce(() -> this.resetPIDControllersReef());
    }

    public Command pigeonResetCommand () {
      return runOnce(() -> this.getPigeon2().reset());
    }

    //Reef Pose Reset
    public Command poseResetCommandReef () {
      return runOnce(() -> this.poseEstimatorReef.resetPose(new Pose2d(0, 0, new Rotation2d(0))));
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                            //Driving to Tag Left//
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    // goalPose = targetPose.transformBy(
    //   new Transform3d( 
    //     new Translation3d(1, 0, 0), 
    //     new Rotation3d(0, 0, Math.toRadians(180))
    //   )
    // ).toPose2d();


    public double run_xControllerLeft() {

        //Gets the pose of the robot
        var robotPose2d = this.getCurrentPoseReef();
        var robotPose = new Pose3d(
          robotPose2d.getX(),
          robotPose2d.getY(),
          0,
          new Rotation3d(0, 0, this.getCurrentPoseReef().getRotation().getRadians())
        );

        //Changes targetPose to whatever April tag is seen
        var targetPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        if (LimelightHelpers.getFiducialID("limelight-backup") == 6) {
          targetPose = AprilTagConstants.aprilTag6_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 7) {
          targetPose = AprilTagConstants.aprilTag7_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 8) {
          targetPose = AprilTagConstants.aprilTag8_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 9) {
          targetPose = AprilTagConstants.aprilTag9_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 10) {
          targetPose = AprilTagConstants.aprilTag10_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 11) {
          targetPose = AprilTagConstants.aprilTag11_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 17) {
          targetPose = AprilTagConstants.aprilTag17_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 18) {
          targetPose = AprilTagConstants.aprilTag18_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 19) {
          targetPose = AprilTagConstants.aprilTag19_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 20) {
          targetPose = AprilTagConstants.aprilTag20_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 21) {
          targetPose = AprilTagConstants.aprilTag21_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 22) {
          targetPose = AprilTagConstants.aprilTag22_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else {
          targetPose = null;
          goalPose = null;
        }

        //Sets the goal of the PID Controller to whatever the goalPose is
        try {
          xController.setGoal(goalPose.getX());
        }
        catch (Exception e) {
          //System.out.print("TargetPose and GoalPose are null");
        }

        //Profiled PIDs //Check this portion of code later to make sure it works
        var xSpeed = xController.calculate(robotPose.getX());
        double xSpeedCheck;

        if (xController.atGoal()) {
          xSpeedCheck = 0;
        }
        else {
          xSpeedCheck = 1;
        }
        
        try {
          SmartDashboard.putBoolean("X At Goal?", xController.atGoal());
          SmartDashboard.putNumber("xGoalPose", goalPose.getX());
          SmartDashboard.putNumber("xtargetPose", targetPose.getX());
          SmartDashboard.putNumber("xSpeed", xSpeed * MaxSpeed * xSpeedCheck);
        } catch (Exception e) {
          //System.out.println("GoalPose returning null");
        }
       
        return xSpeed * MaxSpeed * xSpeedCheck;

    }

    public double run_yControllerLeft() {
    
        //Gets the pose of the robot
        var robotPose2d = this.getCurrentPoseReef();
        var robotPose = new Pose3d(
          robotPose2d.getX(),
          robotPose2d.getY(),
          0,
          new Rotation3d(0, 0, this.getCurrentPoseReef().getRotation().getRadians())
        );
    
        //Changes targetPose to whatever April tag is seen
        var targetPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
    
        if (LimelightHelpers.getFiducialID("limelight-backup") == 6) {
          targetPose = AprilTagConstants.aprilTag6_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 7) {
          targetPose = AprilTagConstants.aprilTag7_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 8) {
          targetPose = AprilTagConstants.aprilTag8_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 9) {
          targetPose = AprilTagConstants.aprilTag9_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 10) {
          targetPose = AprilTagConstants.aprilTag10_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 11) {
          targetPose = AprilTagConstants.aprilTag11_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 17) {
          targetPose = AprilTagConstants.aprilTag17_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 18) {
          targetPose = AprilTagConstants.aprilTag18_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 19) {
          targetPose = AprilTagConstants.aprilTag19_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 20) {
          targetPose = AprilTagConstants.aprilTag20_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 21) {
          targetPose = AprilTagConstants.aprilTag21_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 22) {
          targetPose = AprilTagConstants.aprilTag22_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else {
           targetPose = null;
           goalPose = null;
        }
    
        //Sets the goal of the PID Controller to whatever the goalPose is
        try {
          yController.setGoal(goalPose.getY());
        }
        catch (Exception e) {
          //System.out.print("TargetPose and GoalPose are null");
        }

        //Profiled PIDs //Check this portion of code later to make sure it works
        var ySpeed = yController.calculate(robotPose.getY());
        double ySpeedCheck;
    
        if (yController.atGoal()) {
          ySpeedCheck = 0;
        }
        else {
          ySpeedCheck = 1;
        }

        try {
          SmartDashboard.putBoolean("Y At Goal?", yController.atGoal());
          SmartDashboard.putNumber("yGoalPose", goalPose.getY());
          SmartDashboard.putNumber("ySpeed", ySpeed * MaxSpeed * ySpeedCheck);
          SmartDashboard.putNumber("ytargetPose", targetPose.getY());

        } catch (Exception e) {
          //System.out.println("GoalPose returning null");
        }
        
        return ySpeed * MaxSpeed * ySpeedCheck;
    
    }

    public double run_omegaControllerLeft() {
    
        //Gets the pose of the robot
        var robotPose2d = this.getCurrentPoseReef();
        
        //Changes targetPose to whatever April tag is seen
        var targetPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
        
        if (LimelightHelpers.getFiducialID("limelight-backup") == 6) {
          targetPose = AprilTagConstants.aprilTag6_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 7) {
          targetPose = AprilTagConstants.aprilTag7_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 8) {
          targetPose = AprilTagConstants.aprilTag8_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 9) {
          targetPose = AprilTagConstants.aprilTag9_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 10) {
          targetPose = AprilTagConstants.aprilTag10_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 11) {
          targetPose = AprilTagConstants.aprilTag11_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 17) {
          targetPose = AprilTagConstants.aprilTag17_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 18) {
          targetPose = AprilTagConstants.aprilTag18_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 19) {
          targetPose = AprilTagConstants.aprilTag19_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 20) {
          targetPose = AprilTagConstants.aprilTag20_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 21) {
          targetPose = AprilTagConstants.aprilTag21_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else if (LimelightHelpers.getFiducialID("limelight-backup") == 22) {
          targetPose = AprilTagConstants.aprilTag22_Position;
          goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Left).toPose2d();
        }
        else {
          targetPose = null;
          goalPose = null;
        }
        
        //Sets the goal of the PID Controller to whatever the goalPose is
        try {
          omegaController.setGoal(goalPose.getRotation().getRadians());
        }
        catch (Exception e) {
          //System.out.print("TargetPose and GoalPose are null");
        }
    
        //Profiled PIDs //Check this portion of code later to make sure it works
        var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
        double omegaSpeedCheck;
        
        if (omegaController.atGoal()) {
          omegaSpeedCheck = 0;
        }
        else {
          omegaSpeedCheck = 1;
        }
            
        try {
          SmartDashboard.putBoolean("Omega At Goal?", omegaController.atGoal());
          SmartDashboard.putNumber("omegaGoalPose", goalPose.getRotation().getDegrees());
          SmartDashboard.putNumber("omegaSpeed", omegaSpeed * MaxSpeed * omegaSpeedCheck);
        } catch (Exception e) {
          //System.out.println("GoalPose returns null");
        }
     
        return omegaSpeed * MaxAngularRate * omegaSpeedCheck;
        
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                      
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                                                    //Driving to Tag Right//
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public double run_xControllerRight() {
    
      //Gets the pose of the robot
      var robotPose2d = this.getCurrentPoseReef();
      var robotPose = new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0,
        new Rotation3d(0, 0, this.getCurrentPoseReef().getRotation().getRadians())
      );

      //Changes targetPose to whatever April tag is seen
      var targetPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
      var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

      if (LimelightHelpers.getFiducialID("limelight-backup") == 6) {
        targetPose = AprilTagConstants.aprilTag6_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 7) {
        targetPose = AprilTagConstants.aprilTag7_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 8) {
        targetPose = AprilTagConstants.aprilTag8_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 9) {
        targetPose = AprilTagConstants.aprilTag9_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 10) {
        targetPose = AprilTagConstants.aprilTag10_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 11) {
        targetPose = AprilTagConstants.aprilTag11_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 17) {
        targetPose = AprilTagConstants.aprilTag17_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 18) {
        targetPose = AprilTagConstants.aprilTag18_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 19) {
        targetPose = AprilTagConstants.aprilTag19_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 20) {
        targetPose = AprilTagConstants.aprilTag20_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 21) {
        targetPose = AprilTagConstants.aprilTag21_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 22) {
        targetPose = AprilTagConstants.aprilTag22_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else {
        targetPose = null;
        goalPose = null;
      }

      //Sets the goal of the PID Controller to whatever the goalPose is
      try {
        xController.setGoal(goalPose.getX());
      }
      catch (Exception e) {
        //System.out.print("TargetPose and GoalPose are null");
      }

      //Profiled PIDs //Check this portion of code later to make sure it works
      var xSpeed = xController.calculate(robotPose.getX());
      double xSpeedCheck;

      if (xController.atGoal()) {
        xSpeedCheck = 0;
      }
      else {
        xSpeedCheck = 1;
      }
      
      try {
        SmartDashboard.putBoolean("X At Goal?", xController.atGoal());
        SmartDashboard.putNumber("xGoalPose", goalPose.getX());
        SmartDashboard.putNumber("xtargetPose", targetPose.getX());
        SmartDashboard.putNumber("xSpeed", xSpeed * MaxSpeed * xSpeedCheck);
      } catch (Exception e) {
        //System.out.println("GoalPose returning null");
      }
     
      return xSpeed * MaxSpeed * xSpeedCheck;

  }

  public double run_yControllerRight() {
  
      //Gets the pose of the robot
      var robotPose2d = this.getCurrentPoseReef();
      var robotPose = new Pose3d(
        robotPose2d.getX(),
        robotPose2d.getY(),
        0,
        new Rotation3d(0, 0, this.getCurrentPoseReef().getRotation().getRadians())
      );
  
      //Changes targetPose to whatever April tag is seen
      var targetPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
      var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
      int aprilTagID = -1;
  
      if (LimelightHelpers.getFiducialID("limelight-backup") == 6) {
        targetPose = AprilTagConstants.aprilTag6_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 7) {
        targetPose = AprilTagConstants.aprilTag7_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 8) {
        targetPose = AprilTagConstants.aprilTag8_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 9) {
        targetPose = AprilTagConstants.aprilTag9_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 10) {
        targetPose = AprilTagConstants.aprilTag10_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 11) {
        targetPose = AprilTagConstants.aprilTag11_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 17) {
        targetPose = AprilTagConstants.aprilTag17_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 18) {
        targetPose = AprilTagConstants.aprilTag18_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 19) {
        targetPose = AprilTagConstants.aprilTag19_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 20) {
        targetPose = AprilTagConstants.aprilTag20_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 21) {
        targetPose = AprilTagConstants.aprilTag21_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 22) {
        targetPose = AprilTagConstants.aprilTag22_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else {
         targetPose = null;
         goalPose = null;
      }
  
      //Sets the goal of the PID Controller to whatever the goalPose is
      try {
        yController.setGoal(goalPose.getY());
      }
      catch (Exception e) {
        //System.out.print("TargetPose and GoalPose are null");
      }

      //Profiled PIDs //Check this portion of code later to make sure it works
      var ySpeed = yController.calculate(robotPose.getY());
      double ySpeedCheck;
  
      if (yController.atGoal()) {
        ySpeedCheck = 0;
      }
      else {
        ySpeedCheck = 1;
      }

      try {
        SmartDashboard.putBoolean("Y At Goal?", yController.atGoal());
        SmartDashboard.putNumber("yGoalPose", goalPose.getY());
        SmartDashboard.putNumber("ySpeed", ySpeed * MaxSpeed * ySpeedCheck);
        SmartDashboard.putNumber("ytargetPose", targetPose.getY());

      } catch (Exception e) {
        //System.out.println("GoalPose returning null");
      }
      
      return ySpeed * MaxSpeed * ySpeedCheck;
  
  }

  public double run_omegaControllerRight() {
  
      //Gets the pose of the robot
      var robotPose2d = this.getCurrentPoseReef();

      //Changes targetPose to whatever April tag is seen
      var targetPose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
      var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();
      
      if (LimelightHelpers.getFiducialID("limelight-backup") == 6) {
        targetPose = AprilTagConstants.aprilTag6_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 7) {
        targetPose = AprilTagConstants.aprilTag7_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 8) {
        targetPose = AprilTagConstants.aprilTag8_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 9) {
        targetPose = AprilTagConstants.aprilTag9_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 10) {
        targetPose = AprilTagConstants.aprilTag10_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 11) {
        targetPose = AprilTagConstants.aprilTag11_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 17) {
        targetPose = AprilTagConstants.aprilTag17_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 18) {
        targetPose = AprilTagConstants.aprilTag18_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 19) {
        targetPose = AprilTagConstants.aprilTag19_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 20) {
        targetPose = AprilTagConstants.aprilTag20_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 21) {
        targetPose = AprilTagConstants.aprilTag21_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else if (LimelightHelpers.getFiducialID("limelight-backup") == 22) {
        targetPose = AprilTagConstants.aprilTag22_Position;
        goalPose = targetPose.transformBy(autoAlignTransformConstants.reefGoalPoseTransform_Right).toPose2d();
      }
      else {
        targetPose = null;
        goalPose = null;
      }
      
      //Sets the goal of the PID Controller to whatever the goalPose is
      try {
        omegaController.setGoal(goalPose.getRotation().getRadians());
      }
      catch (Exception e) {
        //System.out.print("TargetPose and GoalPose are null");
      }
  
      //Profiled PIDs //Check this portion of code later to make sure it works
      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());
      double omegaSpeedCheck;
      
      if (omegaController.atGoal()) {
        omegaSpeedCheck = 0;
      }
      else {
        omegaSpeedCheck = 1;
      }
          
      try {
        SmartDashboard.putBoolean("Omega At Goal?", omegaController.atGoal());
        SmartDashboard.putNumber("omegaGoalPose", goalPose.getRotation().getDegrees());
        SmartDashboard.putNumber("omegaSpeed", omegaSpeed * MaxSpeed * omegaSpeedCheck);
      } catch (Exception e) {
        //System.out.println("GoalPose returns null");
      }
   
      return omegaSpeed * MaxAngularRate * omegaSpeedCheck;
      
  }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        //58.5
        //SmartDashboard.putNumber("distance", (55.25 - 9.25) / 
        //Math.tan(Math.toRadians(25.5) + Math.toRadians (LimelightHelpers.getTY("limelight-backup"))));

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        //Reef Pose Estimation and Limelight Stuff
        try {
        SmartDashboard.putNumber("Pos: X", poseEstimatorReef.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Pos: Y", poseEstimatorReef.getEstimatedPosition().getY());
        SmartDashboard.putNumber("Pos: Rot", poseEstimatorReef.getEstimatedPosition().getRotation().getDegrees());
        }
        catch (Exception e) {
          //System.out.println("Nothing being recieved from limelight");
        }

        this.updateOdometryReef();
        SmartDashboard.putNumber("limelight reef ID", LimelightHelpers.getFiducialID("limelight-backup"));

        /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }
}