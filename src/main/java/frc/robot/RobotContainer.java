// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Library.ImprovedCommandXboxController;
import frc.robot.commands.AimAprilTagCmd;
import frc.robot.commands.FeedingCmd;
import frc.robot.commands.SlowExtenderCmd;
import frc.robot.commands.fineTuneDrivetrainCmd;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Extender;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TurrentSystem;
import frc.robot.subsystems.FeedingSubsystem;
import frc.robot.utils.SmartDashboardEx;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final TurrentSystem turrentSystem = new TurrentSystem();
    public final Extender extender = new Extender();
    public final Shooter shooter = new Shooter();
    public final FeedingSubsystem m_feedingSubsystem = new FeedingSubsystem();
    private StructArrayPublisher<SwerveModuleState> swerveStatePublisher;

    public static final ImprovedCommandXboxController m_driverController = new ImprovedCommandXboxController(0);
    public static final ImprovedCommandXboxController m_driverController2 = new ImprovedCommandXboxController(1);
    public static final ImprovedCommandXboxController m_driverController3 = new ImprovedCommandXboxController(2);

    Command m_autoPath;

    private List<Trigger> pathplannerEvents = new ArrayList<Trigger>();


    private final StructPublisher<Pose2d> robotPospublisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();

    private final double HEADING_RED = 0;
    private final double HEADING_BLUE = 180;

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());

        swerveStatePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("/MyStates", SwerveModuleState.struct).publish();

        drivetrain.setDefaultCommand(
            drivetrain.run(() -> drivetrain.driveFieldCentric(m_driverController))
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // setHeading here for auto 
        
        double headingAngle = HEADING_BLUE;  // HEADING_RED or HEADING_BLUE;\
        if (Constants.alliance == Alliance.Red) {
            headingAngle = HEADING_RED;
        }
        // // var alliance = DriverStation.getAlliance();
        // // if (alliance.isPresent()) {
        // //     switch (alliance.get()) {
        // //         case Blue:
        // //         headingAngle = 180;
        // //         break;
        // //         case Red:
        // //         headingAngle = 0;
        // //         break;
        // //     }
        // // }
        drivetrain.resetHeadingForOdo(headingAngle);

        configureDriver1Bindings();
        configureDriver2Bindings();
        registerPathplannerEventsAndNamedCommands();

        drivetrain.registerTelemetry(logger::telemeterize);
        
        // m_autoPath = new PathPlannerAuto("Blue3CLLS");
    }

    
    private void configureDriver1Bindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // m_driverController.a().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // m_driverController.b().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        // m_driverController.x().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // m_driverController.y().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // m_driverController.rightBumper().onTrue(new InstantCommand(() -> {
        //     logger.flush();
        // }));


        // reset the field-centric heading on left bumper press
        // 设头
        m_driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.setFieldCentric()));
        
        // 角度归零，应该让机器正面朝向红方场地，然后按这个按钮，使得机器的0°正对着红方场地。对视觉定位有重要影响
        m_driverController.start().onTrue(new InstantCommand(() -> {
            drivetrain.resetHeadingForOdo(0);
        }));

        // 机器移动微调，前后左右平移
        m_driverController.povUp().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 0));
        m_driverController.povLeft().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 1));
        m_driverController.povDown().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 2));
        m_driverController.povRight().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 3));

        // 机器旋转微调
        m_driverController.leftTrigger().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 4));
        m_driverController.rightTrigger().whileTrue(new fineTuneDrivetrainCmd(drivetrain, 5));
        // m_driverController.povUp() 

        // Vision-assisted aiming while holding right bumper: keep X/Y from driver,
        // but use vision to compute rotation (DriveWithAim will run while held).
        m_driverController.rightBumper().whileTrue(
            new AimAprilTagCmd(drivetrain, 
                            turrentSystem, 
                            false)
        );
        // m_driverController.a().onTrue(new SlowExtenderCmd(extender, Extender.Position.IN.motorPosition()));
        // m_driverController.b().onTrue(new SlowExtenderCmd(extender, Extender.Position.OUT.motorPosition()));
        // m_driverController.x().whileTrue(new InstantCommand(() -> extender.setPosition(Extender.Position.IN)));
        // m_driverController.y().whileTrue(new InstantCommand(() -> extender.setPosition(Extender.Position.OUT)));

        m_driverController.a().whileTrue(new FeedingCmd(m_feedingSubsystem));
    }

    private void configureDriver2Bindings() {
        m_driverController2.x().whileTrue(new InstantCommand(() -> shooter.setPercentOutput(0.9)));
        m_driverController2.y().whileTrue(new InstantCommand(() -> shooter.stop()));
    }

    private void configureDriver3Bindings() {
        // m_driverController3
    }

    private void registerPathplannerEventsAndNamedCommands() {
        
    }

    private void regPPEnC(String name, Runnable runnable) {
        NamedCommands.registerCommand(name, new InstantCommand(runnable));
        pathplannerEvents.add(new EventTrigger(name).onTrue(new InstantCommand(runnable)));
    }

    private void regPPEnC(String name, Command cmd) {
        NamedCommands.registerCommand(name, cmd);
        pathplannerEvents.add(new EventTrigger(name).onTrue(cmd));
    }

    public Command getAutonomousCommand() {
        return m_autoPath;
    }

    public void update() {
        Pose2d pos = drivetrain.getPose();

        swerveStatePublisher.set(drivetrain.getModuleStates());
        robotPospublisher.set(pos);
    }

    public void updateAlways() {
    }


    public void telInit() {
        // LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_LEFT, 0);
        // LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT, 0);
        // // LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_OBJECT_DETECTION, 0);

    }

    public void autoInit() {
        // LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_LEFT, 0);
        // LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT, 0);
        // // LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_OBJECT_DETECTION, 0);
    }

    public void testInit() {
        configureDriver3Bindings();
    }

    public void onDisabled() {
        SmartDashboardEx.flush();
        // if (GlobalConfig.devMode) {
        //     LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_RIGHT, 2);
        //     LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_ARPIL_TAG_NAME_LEFT, 2);
        //     // LimelightHelpers.setPipelineIndex(Constants.LIME_LIGHT_OBJECT_DETECTION, 2);
        // }
    }
}
