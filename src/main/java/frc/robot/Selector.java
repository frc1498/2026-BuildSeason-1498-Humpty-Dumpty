package frc.robot;

import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.Iterator;
import java.util.function.Supplier;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.MotorEnableConstants;

public class Selector extends SubsystemBase {

    private int currentSelection;
    private String currentSelectionName;
    private ArrayList<String> selections;
    private String filterCriteria = "";

    // Fall back to a default of no telemetry.
    private MotorEnableConstants.TelemetryLevel telemetryLevel = MotorEnableConstants.TelemetryLevel.NONE;

    /**
     * Basic constructor.
     */
    public Selector() {
        this.currentSelection = 0;
        this.currentSelectionName = "";
        this.selections = new ArrayList<String>();
    }

    /**
     * Constructor with a predefined list passed in as a parameter.
     * @param selections
     */
    public Selector(ArrayList<String> selections) {
        this();
        this.selections = selections;
        this.sortSelections();
    }

    public Selector(ArrayList<String> selections, String name) {
        this();
        this.selections = this.addSelectionList(selections);
        this.sortSelections();
        this.setSmartDashboardName(name);
    }

    public Selector(File folderPath) {
        this();
        this.selections = addSelectionList(folderPath);
        this.sortSelections();
    }

    public Selector(File folderPath, String extension) {
        this();
        this.selections = this.addSelectionList(folderPath, extension);
        this.sortSelections();
    }

    public Selector(File folderPath, String extension, String name, MotorEnableConstants.TelemetryLevel telemetryLevel) {
        this();
        this.telemetryLevel = telemetryLevel;
        this.selections = this.addSelectionList(folderPath, extension);
        this.sortSelections();
        this.setSmartDashboardName(name);
    }

    private void setSmartDashboardName(String name) {
        SmartDashboard.putData(name, this);
    }

    private ArrayList<String> addSelectionList(ArrayList<String> selectionList) {
        return selectionList;
    }

    private ArrayList<String> addSelectionList(File folderPath) {
        ArrayList<String> placeholder = new ArrayList<String>();
        for (var i : folderPath.listFiles()) {
            if(i.isFile()) {
                placeholder.add(i.getName());
            }
        }
        return placeholder;
    }

    private ArrayList<String> removeSubstringFromList(ArrayList<String> list, String substring) {
        ArrayList<String> placeholder = new ArrayList<String>();
        for (var i: list) {
            placeholder.add(i.replaceFirst(substring, ""));
        }
        return placeholder;
    }

    private ArrayList<String> addSelectionList(File folderPath, String extension) {
        return this.removeSubstringFromList(this.addSelectionList(folderPath), extension);
    }

    private ArrayList<String> getSelectionList() {
        return this.selections;
    }

    public String getCurrentSelectionName() {
        return this.currentSelectionName;
    }

    private void setCurrentSelectionName() {
        this.currentSelectionName = this.selections.get(this.currentSelection);
    }

    private int getCurrentIndex() {
        return this.currentSelection;
    }

    private void setSelection(int index) {
        if (index < 0) {
            this.currentSelection = 0;
        }
        else if (index > this.selections.size() - 1) {
            this.currentSelection = this.selections.size();
        }
        else {
            this.currentSelection = index;
        }
    }

    private void decrementSelection() {
        if (this.currentSelection > 0) {
            this.currentSelection--;
        }
        else if (this.currentSelection < 0) {
            this.currentSelection = 0;
        }
        else {
            this.currentSelection = this.selections.size() - 1;
        }
    }

    private void incrementSelection() {
        if (this.currentSelection < this.selections.size() - 1) {
            this.currentSelection++;
        }
        else if (this.currentSelection > this.selections.size() - 1) {
            this.currentSelection = this.selections.size() - 1;
        }
        else {
            this.currentSelection = 0;
        }
    }

    private ArrayList<String> filterSelections(ArrayList<String> list, Supplier<String> filterCriteria) {
        Iterator<String> filter = list.iterator();
        ArrayList<String> placeholder = new ArrayList<String>();
        String toCheck;

        this.filterCriteria = filterCriteria.get();

        while(filter.hasNext()) {
            toCheck = filter.next();
            if (toCheck.contains(filterCriteria.get())) {
                placeholder.add(toCheck);
            }
        }
        return placeholder;
    }

    private void sortSelections() {
         this.selections.sort(Comparator.naturalOrder());
         this.setCurrentSelectionName();
    }

    private String getCurrentCommandName() {
        if (this.getCurrentCommand() == null) {
            return "No Command";
        }
        else {
            return this.getCurrentCommand().getName();
        }
    }

    //For debugging only.
    private String[] getCurrentListArray() {
        return this.getSelectionList().toArray(new String[this.getSelectionList().size()]);
    }

    public Command filterList(Supplier<String> criteria) {
        return runOnce(
            () -> {
                this.selections = filterSelections(this.selections, criteria);
                this.sortSelections();
                this.setSelection(0);
                this.setCurrentSelectionName();
            }
        ).ignoringDisable(true).withName("filterList");
    }

    public Command increment() {
        return runOnce(
            () -> {
                this.incrementSelection();
                this.setCurrentSelectionName();
            }
        ).ignoringDisable(true).withName("increment");
    }

    public Command decrement() {
        return runOnce(
            () -> {
                this.decrementSelection();
                this.setCurrentSelectionName();
            }
        ).ignoringDisable(true).withName("decrement");
    }

    public Supplier<ArrayList<String>> currentList() {
        return this::getSelectionList;
    }

    public Supplier<Integer> currentIndex() {
        return this::getCurrentIndex;
    }

    public void initSendable(SendableBuilder builder) {
        // I want to use a quirk of switch statements.  If a case doesn't have a break statement, the code below it will continue to run.
        // That can be used to 'gate' values to log without lines of identical code.
        switch (this.telemetryLevel) {
        case FULL:
            builder.addStringProperty("Filter String", () -> {return this.filterCriteria;}, null);
            builder.addIntegerProperty("Number of Selections", () -> {return this.selections.size();}, null);
            builder.addStringArrayProperty("Current List", this::getCurrentListArray, null);
        case LIMITED:
            builder.addStringProperty("Current Command", this::getCurrentCommandName, null);
            builder.addStringProperty("Current Selection", this::getCurrentSelectionName, null);
        case NONE:
            // No values!
        default:
            break;
        } 
    }
}
