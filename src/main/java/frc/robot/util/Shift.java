package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shift extends SubsystemBase{
    private static Alliance shift;
    private static Timer matchTimer = new Timer();

    public static void startMatchTime() {
        matchTimer.start();
    }

    public static void setShift(char gameData) {
        double timeSec = matchTimer.getTimestamp();
        Alliance currentAlliance = KnownLocations.getKnownLocations().alliance;
        Alliance otherAlliance = currentAlliance == Alliance.Blue ? Alliance.Red : Alliance.Blue;

        if(timeSec < 20.0) {
            // AUTO
            shift = currentAlliance;
        } else if(timeSec < 30.0) {
            // TRANSITION
            shift = currentAlliance;
        } else if(timeSec < 55.0) {
            shift = gameData == 'B' ? otherAlliance : currentAlliance; 
        } else if(timeSec < 80.0) {
            shift = gameData == 'B' ? currentAlliance : otherAlliance; 
        } else if(timeSec < 105.0) {
            shift = gameData == 'B' ? otherAlliance : currentAlliance;
        } else if(timeSec < 130.0) {
            shift = gameData == 'B' ? currentAlliance : otherAlliance; 
        } else if(timeSec < 160.0) {
            shift = currentAlliance;
        }
    }

    public static boolean canShoot() {
        return shift == KnownLocations.getKnownLocations().alliance;
    }

    public static void setShiftWithButtons() {

    }

    public void periodic() {
        String gameData;
        gameData = DriverStation.getGameSpecificMessage();
        if (gameData.length() > 0) {
            switch (gameData.charAt(0)) {
                case 'B':
                    // Blue case code
                    break;
                case 'R':
                    // Red case code
                    break;
                default:
                    // This is corrupt data
                    break;
            }
        } else {
            // Code for no data received yet
        }
    }
}
