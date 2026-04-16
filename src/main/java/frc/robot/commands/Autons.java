package frc.robot.commands;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.sensors.DistanceSensors;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.hopper.Indexer;
import frc.robot.subsystems.hopper.Indexer.IndexerSpeed;
import frc.robot.subsystems.intake.Articulator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.outtake.Hood;
import frc.robot.subsystems.outtake.Kicker;
import frc.robot.subsystems.outtake.Kicker.KickerSpeed;
import frc.robot.subsystems.outtake.Shooter;
import frc.robot.util.KnownLocations;
import frc.robot.util.PathUtils;
import frc.robot.util.TargetUtils;

public class Autons {

    private SendableChooser<AutonType> autonTypeChooser;
    // private SendableChooser<Pose2d> startingPoseChooser;


    private AutonType autoType = AutonType.PRE_LOAD_RIGHT;

    private Command autonCommand;

    private Alliance alliance;

    private final double ROTATION_P = 3.0;
    private final double TRANSLATION_P = 5.0;

    private final Command DO_NOTHING = new PrintCommand("Do Nothing Auton");
    private Drivetrain drivetrain;
    // private DistanceSensors proximitySensor;
    private RobotConfig config;
    private Shooter shooter;
    private Indexer indexer;
    private Kicker kicker;
    private Hood hood;
    private Articulator articulator;

    public Autons(Drivetrain drivetrain, Hood hood, Shooter shooter, Indexer indexer, Kicker kicker, Articulator articulator) {
        // TODO: Put correct settings into PathPlanner GUI for the new robot
        this.drivetrain = drivetrain;
        this.shooter = shooter;
        this.indexer = indexer;
        this.kicker = kicker;
        this.hood = hood;
        this.articulator = articulator;

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
                () -> drivetrain.getFieldRelativeSpeeds(), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
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

    public void displayPaths(List<PathPlannerPath> paths) {
        boolean isRedAlliance = (KnownLocations.getKnownLocations().alliance == Alliance.Red);
        int trajIndex = 0;

        for (int i = 0; i < paths.size(); i++) {
                PathPlannerPath path = isRedAlliance ? paths.get(i).flipPath() : paths.get(i);
                drivetrain.setTrajectorySmartdash(PathUtils.TrajectoryFromPath(path, config), trajIndex + "");
            trajIndex++;
        }
    }


    public Command buildAutonCommand(KnownLocations knownLocations) {
        // TODO: Add this logic in KnownLocations, and make sure to update based on alliance changes
        // drivetrain.resetPose(startingPose);
        // drivetrain.setStartingPosition(startingPose); Do we need this?
        SequentialCommandGroup autonCommand = new SequentialCommandGroup();

        ArrayList<PathPlannerPath> paths = new ArrayList<>();
        
        BooleanSupplier shooting = () -> shooter.getShootingTrigger() && hood.isInPosition() /* && drivetrain.isPointingAtVector() */ && drivetrain.isDrivetrainInAllianceZone();

        SequentialCommandGroup shootCommand = new SequentialCommandGroup(
                DriveCommands.lockToHub(() -> 0.0, () -> 0.0, drivetrain).until(shooting),
                new ParallelCommandGroup(
                        // new RunCommand(() -> hood.setPosition(0.0), hood),
                        DriveCommands.lockToHub(() -> 0.0, () -> 0.0, drivetrain),
                        RobotCommands.feedShooter(indexer, kicker),
                        RobotCommands.agitateIntake(articulator)
                    ));
        
        SequentialCommandGroup shootCommand2 = new SequentialCommandGroup(
                DriveCommands.lockToHub(() -> 0.0, () -> 0.0, drivetrain).until(shooting),
                new ParallelCommandGroup(
                        // new RunCommand(() -> hood.setPosition(0.0), hood),
                        DriveCommands.lockToHub(() -> 0.0, () -> 0.0, drivetrain),
                        RobotCommands.feedShooter(indexer, kicker),
                        RobotCommands.agitateIntake(articulator)
        ));

        switch (autoType) {
            case SINGLE_SWIPE_LEFT: // DONE
                try {
                    paths.add(PathPlannerPath.fromPathFile("leftStartToOutsideSweepToLeftShoot"));
                    autonCommand.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("leftStartToOutsideSweepToLeftShoot")));
                } catch (Exception e) {
                    
                }
                break;
            case PRE_LOAD_MIDDLE:
                try {
                    paths.add(PathPlannerPath.fromPathFile("middleStartToMiddleShoot"));
                    autonCommand.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("middleStartToMiddleShoot")));
                    autonCommand.addCommands(shootCommand);
                } catch (Exception e) {
                
                }
                break;
            case OUTPOST_MIDDLE:
                try {
                    paths.add(PathPlannerPath.fromPathFile("middleStartToOutpost"));
                    paths.add(PathPlannerPath.fromPathFile("outpostToShoot"));
                    autonCommand.addCommands(
                        new WaitCommand(0.0),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("middleStartToOutpost")),
                        new WaitCommand(5.0),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("outpostToShoot")),
                        shootCommand
                    );
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }
                break;
            case SINGLE_SWIPE_RIGHT: // DONE
                try {
                    paths.add(PathPlannerPath.fromPathFile("rightStartToOutsideSweepToRightShoot"));
                    autonCommand.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("rightStartToOutsideSweepToRightShoot")));
                    autonCommand.addCommands(shootCommand);
                } catch (Exception e) {
                
                }
                break;
            case PRE_LOAD_LEFT: // DONE
                try {
                    paths.add(PathPlannerPath.fromPathFile("leftStartToShoot"));
                    autonCommand.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("leftStartToShoot")));
                    autonCommand.addCommands(shootCommand);
                } catch (Exception e) {
                
                }
                break;
            case PRE_LOAD_RIGHT: // DONE 
                try {
                    paths.add(PathPlannerPath.fromPathFile("rightStartToShoot"));
                    autonCommand.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("rightStartToShoot")));
                    autonCommand.addCommands(shootCommand);
                } catch (Exception e) {
                
                }
                break;
            case DOUBLE_SWEEP_LEFT: // DONE
                try {
                    paths.add(PathPlannerPath.fromPathFile("leftStartToOutsideSweepToLeftShoot"));
                    paths.add(PathPlannerPath.fromPathFile("leftShootToInsideSweepToLeftShoot"));
                    autonCommand.addCommands(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("leftStartToOutsideSweepToLeftShoot")),
                        shootCommand.withTimeout(4.0),
                        new ParallelRaceGroup(
                            AutoBuilder.followPath(PathPlannerPath.fromPathFile("leftShootToInsideSweepToLeftShoot")),
                            new RunCommand(() -> indexer.setSpeed(IndexerSpeed.STOP), indexer),
                            new RunCommand(() -> kicker.setSpeed(KickerSpeed.STOP), kicker)
                        ),
                        shootCommand2.withTimeout(3.0)
                    );
                    
                   // autonCommand.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("leftStartToInsideSweepToLeftShoot")));
                   // autonCommand.addCommands(shootCommand2);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }
                break;
            case DOUBLE_SWEEP_RIGHT: // DONE
                try {
                    paths.add(PathPlannerPath.fromPathFile("leftStartToOutsideSweepToLeftShoot").mirrorPath());
                    paths.add(PathPlannerPath.fromPathFile("leftShootToInsideSweepToLeftShoot").mirrorPath());
                    autonCommand.addCommands(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("leftStartToOutsideSweepToLeftShoot").mirrorPath()),
                        shootCommand.withTimeout(4.0),
                        new ParallelRaceGroup(
                            AutoBuilder.followPath(PathPlannerPath.fromPathFile("leftShootToInsideSweepToLeftShoot").mirrorPath()),
                            new RunCommand(() -> indexer.setSpeed(IndexerSpeed.STOP), indexer),
                            new RunCommand(() -> kicker.setSpeed(KickerSpeed.STOP), kicker)
                        ),
                        shootCommand2.withTimeout(3.0)
                    );
                    
                   // autonCommand.addCommands(AutoBuilder.followPath(PathPlannerPath.fromPathFile("leftStartToInsideSweepToLeftShoot")));
                   // autonCommand.addCommands(shootCommand2);
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }
                break;
            case SINGLE_SWIPE_RIGHT_AND_OUTPOST: // DONE
                try {
                    paths.add(PathPlannerPath.fromPathFile("leftStartToOutsideSweepToLeftShoot").mirrorPath());
                    paths.add(PathPlannerPath.fromPathFile("rightShootToOutpost"));
                    paths.add(PathPlannerPath.fromPathFile("outpostToShoot"));
                    autonCommand.addCommands(
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("leftStartToOutsideSweepToLeftShoot").mirrorPath()),
                        shootCommand.withTimeout(3.0),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("rightShootToOutpost")),
                        new WaitCommand(3.0),
                        AutoBuilder.followPath(PathPlannerPath.fromPathFile("outpostToShoot")),
                        shootCommand2
                    );
                } catch (Exception e) {
                    System.out.println(e.getMessage());
                }
                break;
            default:
                break;
        }

        try {
            displayPaths(paths);
        } catch (Exception e) {
            // TODO: handle exception
        }

        return autonCommand;
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
        
        if (rebuildAutonCommand) {
            this.autonCommand = buildAutonCommand(knownLocations);
        }
    }

    public void setChoosers(KnownLocations knownLocations) {
        autonTypeChooser = new SendableChooser<AutonType>();
        autonTypeChooser.setDefaultOption("PRE LOAD RIGHT", AutonType.PRE_LOAD_RIGHT);
        // autonTypeChooser.addOption("MIDDLE", AutonType.MIDDLE);
        autonTypeChooser.addOption("PRE LOAD LEFT", AutonType.PRE_LOAD_LEFT);
        autonTypeChooser.addOption("SINGLE SWIPE LEFT", AutonType.SINGLE_SWIPE_LEFT);
        autonTypeChooser.addOption("SINGLE SWIPE RIGHT", AutonType.SINGLE_SWIPE_RIGHT);
        autonTypeChooser.addOption("PRE LOAD MIDDLE", AutonType.PRE_LOAD_MIDDLE);
        autonTypeChooser.addOption("OUTPOST MIDDLE", AutonType.OUTPOST_MIDDLE);
        autonTypeChooser.addOption("DOUBLE SWIPE LEFT", AutonType.DOUBLE_SWEEP_LEFT);
        autonTypeChooser.addOption("DOUBLE SWIPE RIGHT", AutonType.DOUBLE_SWEEP_RIGHT);
        autonTypeChooser.addOption("RIGHT SWIPE + OUTPOST", AutonType.SINGLE_SWIPE_RIGHT_AND_OUTPOST);


        SmartDashboard.putData("Auton Type Chooser", autonTypeChooser);
    }

    public void updateAutonLocations() {
        KnownLocations locs = KnownLocations.getKnownLocations();
    }

    public enum AutonType {
        SINGLE_SWIPE_LEFT, // DONE
        SINGLE_SWIPE_RIGHT, // DONE
        PRE_LOAD_LEFT, // DONE
        PRE_LOAD_MIDDLE,
        OUTPOST_MIDDLE,
        PRE_LOAD_RIGHT, // DONE
        DOUBLE_SWEEP_LEFT, // DONE
        DOUBLE_SWEEP_RIGHT, // DONE
        SINGLE_SWIPE_RIGHT_AND_OUTPOST
    }
}