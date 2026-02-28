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
        if ((DriverStation.getMatchTime() >= 130.0)) {                                                //transition shift - 2:20 - 2:10
            return "Transition Shift";
        }
        else if ((DriverStation.getMatchTime() <= 129.98) && (DriverStation.getMatchTime() >= 105.0)) { //shift 1 - 2:10 - 1:45
            return "Shift 1";
        }
        else if ((DriverStation.getMatchTime() <= 104.98) && (DriverStation.getMatchTime() >= 80.0)) { //shift 2 - 1:45 - 1:20
            return "Shift 2";
        }
        else if ((DriverStation.getMatchTime() <= 79.98) && (DriverStation.getMatchTime() >= 55.0)) { //shift 3 - 1:20 - 0:55
            return "Shift 3";
        }
        else if ((DriverStation.getMatchTime() <= 54.98) && (DriverStation.getMatchTime() >= 30.0)) { //shift 4 - 0:55 - 0:30
            return "Shift 4";
        }
        else {                                                                                     //endgame - 0:30 - 0:00
            return "End Game";
        }
    }

    public DriverStation.Alliance getInitialScoringHub() {
        switch (DriverStation.getGameSpecificMessage()) {
            case "B":
                return DriverStation.Alliance.Blue;
            case "R":
                return DriverStation.Alliance.Red;
            default:
                return DriverStation.Alliance.Blue;
        }
    }

    /**
     * Determine which scoring hub can be used.
     * @return
     */
    public DriverStation.Alliance getCurrentScoringHub() {
        // During the transition shift or the end game, any hub can be scored in, so return the current robot alliance.
        if ((DriverStation.getMatchTime() >= 130.0)) {                                                //transition shift - 2:20 - 2:10
            return DriverStation.getAlliance().get();
        }
        else if ((DriverStation.getMatchTime() <= 129.98) && (DriverStation.getMatchTime() >= 105.0)) { //shift 1 - 2:10 - 1:45
            return this.getInitialScoringHub();
        }
        else if ((DriverStation.getMatchTime() <= 104.98) && (DriverStation.getMatchTime() >= 80.0)) { //shift 2 - 1:45 - 1:20
            if (this.getInitialScoringHub() == DriverStation.Alliance.Blue) {
                return DriverStation.Alliance.Red;
            }
            else {
                return DriverStation.Alliance.Blue;
            }
        }
        else if ((DriverStation.getMatchTime() <= 79.98) && (DriverStation.getMatchTime() >= 55.0)) { //shift 3 - 1:20 - 0:55
            return this.getInitialScoringHub(); //the active hub in shift 3 == active hub from shift 1
        }
        else if ((DriverStation.getMatchTime() <= 54.98) && (DriverStation.getMatchTime() >= 30.0)) { //shift 4 - 0:55 - 0:30
            if (this.getInitialScoringHub() == DriverStation.Alliance.Blue) {
                return DriverStation.Alliance.Red;
            }
            else {
                return DriverStation.Alliance.Blue;
            }
        }
        else {                                                                                     //endgame - 0:30 - 0:00
            return DriverStation.getAlliance().get();
        }
    }

    /**
     * Detects when a shift change is about to happen
     * takes in how many seconds before a shift change you want to be notified
     * @return if your hub will be active in the next shift
     */
    
     public String isNextScoringHubActive(double sec) {
        boolean isNextFlag = true;
        if ((DriverStation.getMatchTime() <= 130.0 + sec) && (DriverStation.getMatchTime() >= 130.0)) { //Next Shift = Shift 1
            isNextFlag = (DriverStation.getAlliance().get() == this.getInitialScoringHub());
        }
        else if ((DriverStation.getMatchTime() <= 105.0 + sec) && (DriverStation.getMatchTime() >= 105.0)) { //Next Shift = Shift 2
            isNextFlag = !(DriverStation.getAlliance().get() == this.getCurrentScoringHub());
        }
        else if ((DriverStation.getMatchTime() <= 80.0 + sec) && (DriverStation.getMatchTime() >= 80.0)) { //Next Shift = Shift 3
            isNextFlag = !(DriverStation.getAlliance().get() == this.getCurrentScoringHub());
        }
        else if ((DriverStation.getMatchTime() <= 55.0 + sec) && (DriverStation.getMatchTime() >= 55.0)) { //Next Shift = Shift 4
            isNextFlag = !(DriverStation.getAlliance().get() == this.getCurrentScoringHub());
        }
        else if ((DriverStation.getMatchTime() <= 30.0 + sec) && (DriverStation.getMatchTime() >= 30.0)) { //Next Shift = End Game
            isNextFlag = true;
        }
        else {                                                                                                                                 
            isNextFlag = false;
        }

        if (isNextFlag == true) {
            return "true";
        }
        else {
            return "false";
        }
     }
}
