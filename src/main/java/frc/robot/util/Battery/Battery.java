package frc.robot.util.Battery;

import com.ctre.phoenix6.SignalLogger;

public class Battery {

    private String id;
    private String buildDate;
    private String purchaseDate;
    private int ranking;
    private double ratedVoltage;
    private double ratedCapacity;
    private double internalResistance;
    private useCase useCase;

    public enum useCase {
        DEVELOPMENT,
        PRACTICE,
        COMPETITION
    }

    public Battery() {
        // Empty Constructor.
        // This is purely a stylistic choice.
    }

    public Battery withID(String id) {
        this.id = id;
        return this;
    }

    public Battery withBuildDate(String buildDate) {
        this.buildDate = buildDate;
        return this;
    }

    public Battery withPurchaseDate(String purchaseDate) {
        this.purchaseDate = purchaseDate;
        return this;
    }

    public Battery withRanking(int ranking) {
        this.ranking = ranking;
        return this;
    }

    public Battery withRatedVoltage(double ratedVoltage) {
        this.ratedVoltage = ratedVoltage;
        return this;
    }

    public Battery withRatedCapacity(double ratedCapacity) {
        this.ratedCapacity = ratedCapacity;
        return this;
    }

    public Battery withInternalResistance(double internalResistance) {
        this.internalResistance = internalResistance;
        return this;
    }

    public Battery withUseCase(useCase useCase) {
        this.useCase = useCase;
        return this;
    }

    public String getID() {return this.id;}
    public String getBuildDate() {return this.buildDate;}
    public String getPurchaseDate() {return this.purchaseDate;}
    public int getRanking() {return this.ranking;}
    public double getRatedVoltage() {return this.ratedVoltage;}
    public double getRatedCapacity() {return this.ratedCapacity;}
    public double getInternalResistance() {return this.internalResistance;}
    public String getUseCase() {return this.useCase.name();}

    public void logMetadata() {
        SignalLogger.writeString("BatteryConstants/ID", this.getID());
        SignalLogger.writeString("BatteryConstants/BuildDate", this.getBuildDate());
        SignalLogger.writeString("BatteryConstants/PurchaseDate", this.getPurchaseDate());
        SignalLogger.writeString("BatteryConstants/UseCase", this.getUseCase());
        SignalLogger.writeInteger("BatteryConstants/Ranking", this.getRanking());
        SignalLogger.writeDouble("BatteryConstants/RatedVoltage", this.getRatedVoltage());
        SignalLogger.writeDouble("BatteryConstants/RatedCapacity", this.getRatedCapacity());
        SignalLogger.writeDouble("BatteryConstants/InternalResistance", this.getInternalResistance());
    }

}
