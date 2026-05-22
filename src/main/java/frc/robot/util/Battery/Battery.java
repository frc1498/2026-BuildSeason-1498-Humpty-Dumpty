package frc.robot.util.Battery;

import com.ctre.phoenix6.SignalLogger;

public class Battery {

    /* Battery Defaults */
    /* These are logged if they are not overwritten with the initialization of a new Battery object. */
    /* This assumes a default MK ES17-12 battery. */
    private String id = "N/A";
    private String manufacturer = "MK";
    private String model = "ES17-12";
    private String buildDate = "N/A";
    private String purchaseDate = "N/A";
    private int ranking = 1;
    private double ratedVoltage = 12.0;
    private double ratedCapacity = 18.0;
    private double internalResistance = 0.012;
    private useCase useCase = useCase.COMPETITION;

    /**
    * A simple enumeration to further rank batteries by their ideal use case.
    * Batteries can be marked for development use, practice use, or competition use.
    */
    public enum useCase {
        DEVELOPMENT,
        PRACTICE,
        COMPETITION
    }

    /**
    * Constructs a new Battery object.
    * The constructor is empty, but data can be added with decorators for each field.
    * The decorators return the object, so they can be chained together.
    * Decorators eliminate the need for constructor overloads.
    */
    public Battery() {
        // Empty Constructor.
        // This is purely a stylistic choice.
    }

    /**
    * Set the ID of the battery.
    * This is the team assigned ID of the battery, whether that's a number or a video game reference.
    * @param id - The team assigned ID of the battery.
    */
    public Battery withID(String id) {
        this.id = id;
        return this;
    }

    /**
    * Set the manufacturer of the battery.
    * There's a 95% chance this is 'MK'.
    * @param manufacturer - The manufacturer of the battery.
    */
    public Battery withManufacturer(String manufacturer) {
        this.manufacturer = manufacturer;
        return this;
    }

    /**
    * Set the model of the battery.
    * Since the battery is probably an MK battery, there's a good chance this is 'ES17-12'.
    * @param model - The model of the battery.
    */
    public Battery withModel(String model) {
        this.model = model;
        return this;
    }

    /**
    * Set the manufacturing date of the battery.
    * In a perfect world, the battery should be relatively new when compared to the date it was purchased.
    * @param buildDate - The date the battery was manufactured, in YYYY-MM-DD format.
    */
    public Battery withBuildDate(String buildDate) {
        this.buildDate = buildDate;
        return this;
    }

    /**
    * Set the purchase date of the battery.
    * This gives a reference of how many seasons the battery has been used for.
    * @param purchaseDate - The date the battery was purchased, in YYYY-MM-DD format.
    */
    public Battery withPurchaseDate(String purchaseDate) {
        this.purchaseDate = purchaseDate;
        return this;
    }

    /**
    * Set the ranking of the battery.
    * This is the team assigned ranking of the battery.
    * 1 would be considered the 'best',
    * @param ranking - The team assigned ranking of the battery.
    */
    public Battery withRanking(int ranking) {
        this.ranking = ranking;
        return this;
    }

    /**
    * Set the rated voltage of the battery.
    * This is assumed to be rated voltage as defined by the manufacturer, but it could also be used for the rated voltage as determined by test results.
    * @param ratedVoltage - The rated voltage of the battery.
    */
    public Battery withRatedVoltage(double ratedVoltage) {
        this.ratedVoltage = ratedVoltage;
        return this;
    }

    /**
    * Set the rated capacity of the battery.
    * This is assumed to be rated capacity as defined by the manufacturer, but it could also be used for the rated capacity as determined by test results.
    * @param ratedCapacity - The rated capacity of the battery, in Amp-Hours.
    */
    public Battery withRatedCapacity(double ratedCapacity) {
        this.ratedCapacity = ratedCapacity;
        return this;
    }

    /**
    * Set the internal resistance of the battery.
    * This is assumed to be the rated internal resistance of the battery, but it could also be used for the current internal resistance as determined by test results and competition usage.
    * @param internalResistance - The internal resistance of the battery, in ohms.
    */
    public Battery withInternalResistance(double internalResistance) {
        this.internalResistance = internalResistance;
        return this;
    }

    /**
    * Set the use case of the battery.
    * This is useful for binning batteries.
    * Occasionally, a battery might not be optimal for competition use, but it perfectly fine for practice.
    * @param useCase - The ideal use case of the battery.
    */
    public Battery withUseCase(useCase useCase) {
        this.useCase = useCase;
        return this;
    }

    // Extermely simple get() methods for all of the class fields.
    // So simple, in fact, that there are no Javadocs.
    public String getID() {return this.id;}
    public String getManufacturer() {return this.manufacturer;}
    public String getModel() {return this.model;}
    public String getBuildDate() {return this.buildDate;}
    public String getPurchaseDate() {return this.purchaseDate;}
    public int getRanking() {return this.ranking;}
    public double getRatedVoltage() {return this.ratedVoltage;}
    public double getRatedCapacity() {return this.ratedCapacity;}
    public double getInternalResistance() {return this.internalResistance;}
    public String getUseCase() {return this.useCase.name();}

    /**
    * Log all of the data associated with this battery.
    * This utilizes the Phoenix 6 logging framework.
    * Ideally, this method should only need to be called once.
    */
    public void logMetadata() {
        SignalLogger.writeString("BatteryConstants/ID", this.getID());
        SignalLogger.writeString("BatteryConstants/Manufacturer", this.getManufacturer());
        SignalLogger.writeString("BatteryConstants/Model", this.getModel());
        SignalLogger.writeString("BatteryConstants/BuildDate", this.getBuildDate());
        SignalLogger.writeString("BatteryConstants/PurchaseDate", this.getPurchaseDate());
        SignalLogger.writeString("BatteryConstants/UseCase", this.getUseCase());
        SignalLogger.writeInteger("BatteryConstants/Ranking", this.getRanking());
        SignalLogger.writeDouble("BatteryConstants/RatedVoltage", this.getRatedVoltage());
        SignalLogger.writeDouble("BatteryConstants/RatedCapacity", this.getRatedCapacity());
        SignalLogger.writeDouble("BatteryConstants/InternalResistance", this.getInternalResistance());
    }

}
