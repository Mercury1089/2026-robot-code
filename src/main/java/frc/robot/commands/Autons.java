package frc.robot.commands;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import frc.robot.sensors.DistanceSensors;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.util.KnownLocations;
import frc.robot.util.PathUtils;
import frc.robot.util.TargetUtils;

public class Autons {

    private SendableChooser<AutonType> autonTypeChooser;
    // private SendableChooser<Pose2d> startingPoseChooser;


    private AutonType autoType = AutonType.RIGHT;

    private Command autonCommand;

    private Alliance alliance;

    private final double ROTATION_P = 3.0;
    private final double TRANSLATION_P = 5.0;

    private final Command DO_NOTHING = new PrintCommand("Do Nothing Auton");
    private Drivetrain drivetrain;
    // private DistanceSensors proximitySensor;
    private RobotConfig config;

    public Autons(Drivetrain drivetrain) {

        this.drivetrain = drivetrain;

        KnownLocations knownLocations = KnownLocations.getKnownLocations();
        this.alliance = knownLocations.alliance;

        setChoosers(knownLocations);
        updateAutonLocations();

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                () -> drivetrain.getPose(), // Robot pose supplier
                (pose) -> drivetrain.resetPose(pose), // Method to reset odometry (will be called if your auto has starting pose)
                () -> drivetrain.getFieldRelativSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (chassisSpeeds) -> drivetrain.drive(chassisSpeeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(TRANSLATION_P, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(ROTATION_P, 0.0, 0.0) // Rotation PID constants
                ),
                config,
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                }, // Never flip a path - all paths use absolute coordinates
                drivetrain // Reference to this subsystem to set requirements
        );
    }

    public Command getAutonCommand() {
        return this.autonCommand;
    }


    public Command buildAutonCommand(KnownLocations knownLocations) {
        return new SequentialCommandGroup(
            
        );
    }

    public PathPlannerPath getBasePath() {
        List<Waypoint> waypoints = new ArrayList<Waypoint>();
        waypoints.add(new Waypoint(new Translation2d(), new Translation2d(), new Translation2d()));
        waypoints.add(new Waypoint(new Translation2d(), new Translation2d(), new Translation2d()));

        PathConstraints constraints = new PathConstraints(0.0,0.0,0.0,0.0,0.0, true);
        IdealStartingState idealStartingState = new IdealStartingState(0.0, Rotation2d.fromDegrees(0.0));
        GoalEndState goalEndState = new GoalEndState(0.0, Rotation2d.fromDegrees(0.0));
        return new PathPlannerPath(waypoints, constraints, idealStartingState, goalEndState);
    }


    /**
     * Rebuilds the autonCommand when ONE of the following conditions changes:
     * - Alliance Color
     * - Starting Pose
     * - like all the things we do
     */

    public void updateDash() {

        boolean rebuildAutonCommand = false;

        KnownLocations knownLocations = KnownLocations.getKnownLocations();

        if (knownLocations.alliance != this.alliance) {
            this.alliance = knownLocations.alliance;
            SmartDashboard.putString("alliance color!", this.alliance.toString());
            setChoosers(knownLocations);
            rebuildAutonCommand = true;
        }

        AutonType type = autonTypeChooser.getSelected();

        if (type != this.autoType) {
            this.autoType = type;
            rebuildAutonCommand = true;
        }

        // Pose2d startingPose = startingPoseChooser.getSelected();
        
        if (rebuildAutonCommand) {
            this.autonCommand = buildAutonCommand(knownLocations);
        }
    }

    public void setChoosers(KnownLocations knownLocations) {
        autonTypeChooser = new SendableChooser<AutonType>();
        autonTypeChooser.setDefaultOption("RIGHT", AutonType.RIGHT);
        autonTypeChooser.addOption("MIDDLE", AutonType.MIDDLE);
        autonTypeChooser.addOption("LEFT", AutonType.LEFT);

        // // select the MANUAL STARTING POSITION of the robot
        // this.startingPoseChooser = new SendableChooser<Pose2d>();
        // startingPoseChooser.setDefaultOption("RIGHT", knownLocations.rightStart);

        // SmartDashboard.putData("Starting Pose", startingPoseChooser);

        SmartDashboard.putData("Auton Type Chooser", autonTypeChooser);
    }

    public void updateAutonLocations() {
        KnownLocations locs = KnownLocations.getKnownLocations();
    }

    public enum AutonType {
        LEFT,
        MIDDLE,
        RIGHT;
    }
}