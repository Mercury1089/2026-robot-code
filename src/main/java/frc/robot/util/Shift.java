package frc.robot.util;

import javax.xml.crypto.dsig.keyinfo.RetrievalMethod;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shift {
    private Alliance shift;

    private boolean isHubActive = true;
    private double timeUntilTransition = 0;

    private String manualAutonWinner = "";

    private class TimeSegment {
        public double start;
        public double end;

        public TimeSegment(double start, double end) {
            this.start = start;
            this.end = end;
        }

        public boolean isTimeWithin(double time) {
            return time >= start && time <= end;
        }
    }

    private final TimeSegment[] TeleopHubActiveTimesForAutoWinner = new TimeSegment[] {
            new TimeSegment(0, 55), // Shift 4 combines with End game
            new TimeSegment((1 * 60) + 20, (1 * 60) + 45), // Shift 2
            new TimeSegment((2 * 60) + 10, (2 * 60) + 20) // Transition Shift
    };
    private final TimeSegment[] TeleopHubActiveTimesForAutoLoser = new TimeSegment[] {
            new TimeSegment(0, 30), // End game
            new TimeSegment(55, (1 * 60) + 20), // Shift 3
            new TimeSegment((1 * 60) + 45, (2 * 60) + 20) // Shift 1 combines with transition shift
    };

    public boolean isOurHubActive() {
        return isHubActive;
    }

    public void setManualAutonWinner(String manualAutonWinner) {
        this.manualAutonWinner = manualAutonWinner;
    }

    public void updateStatesForTeleop() {
        if (!DriverStation.isTeleopEnabled())
            return;

        String whoWonAuto = DriverStation.getGameSpecificMessage();
        boolean redIsWinner;
        switch (whoWonAuto) {
            case "R":
                redIsWinner = true;
                break;
            case "B":
                redIsWinner = false;
                break;
            default:
                if (manualAutonWinner.equals("R")) {
                    redIsWinner = true;
                    break;
                } else if (manualAutonWinner.equals("B")) {
                    redIsWinner = false;
                    break;
                } 
                return;// We don't have the message yet, we can't determine
        }

        /*
         * Check to see if our current time is within a time that it doesn't matter what
         * alliance we're on
         */
        double timeLeftInTeleop = DriverStation.getMatchTime();
        double timeUntilSwap = 150; // Start at 2:30 so it definitely gets cleared

        /*
         * When we're in teleop we should definitely have the alliance color, so we can
         * confidently use the DS alliance API
         */
        boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;

        /*
         * This boils down to an XOR but we explicitly call out both cases so it's clear
         */
        boolean useWinnerTimes = (redIsWinner && isRedAlliance) || (!redIsWinner && !isRedAlliance);

        /* If we're the winner, use the winner times */
        if (useWinnerTimes) {
            for (TimeSegment seg : TeleopHubActiveTimesForAutoWinner) {
                if (seg.isTimeWithin(timeLeftInTeleop)) {
                    timeUntilTransition = timeLeftInTeleop - seg.start;
                    isHubActive = true;
                    return;
                }
                double timeToStart = timeLeftInTeleop - seg.end;
                if (timeToStart > 0 && timeToStart < timeUntilSwap) {
                    /* Update our time until swap with this, since it's sooner */
                    timeUntilSwap = timeToStart;
                }
            }
        } else {
            for (TimeSegment seg : TeleopHubActiveTimesForAutoLoser) {
                if (seg.isTimeWithin(timeLeftInTeleop)) {
                    timeUntilTransition = timeLeftInTeleop - seg.start;
                    isHubActive = true;
                    return;
                }
                double timeToStart = timeLeftInTeleop - seg.end;
                if (timeToStart > 0 && timeToStart < timeUntilSwap) {
                    /* Update our time until swap with this, since it's sooner */
                    timeUntilSwap = timeToStart;
                }
            }
        }
        timeUntilTransition = timeUntilSwap;
        isHubActive = false;
    }
}
