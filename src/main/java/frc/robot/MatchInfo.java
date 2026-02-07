package frc.robot;

import java.text.BreakIterator;

import edu.wpi.first.wpilibj.DriverStation;

public class MatchInfo {
    private static MatchInfo instance;

    /**
     * Return the instance of this class.
     * @return
     */
    public static MatchInfo getInstance() {
        if (instance == null) {instance = new MatchInfo();};
        return instance;
    }

    public double timeLeftinAuton() {
        if (DriverStation.isAutonomousEnabled()) {
            return DriverStation.getMatchTime();
        } else {
            return 0.0;
        }

    }

    /**
     * Returns the current shift name during the match, based on time left in the match.
     * @return
     */
    public String getCurrentShift() {
        return "";
    }

    public DriverStation.Alliance getInitialScoringHub() {
        switch (DriverStation.getGameSpecificMessage()) {
            case "B":
                return DriverStation.Alliance.Blue;
                // break;
            case "R":
                return DriverStation.Alliance.Red;
                // break;
            default:
                return DriverStation.Alliance.Blue;
                // break;
        }
    }

    /**
     * Determine which scoring hub can be used.
     * @return
     */
    public DriverStation.Alliance getCurrentScoringHub() {


        // During the end game, any hub can be scored in, so return the current robot alliance.
        if (DriverStation.getMatchTime() <= 30.0) {
            return DriverStation.getAlliance().get();
        }
    }

}
