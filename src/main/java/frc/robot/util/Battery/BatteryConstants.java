package frc.robot.util.Battery;

import java.util.ArrayList;

import frc.robot.util.Battery.Battery.useCase;

public class BatteryConstants {

    public static final Battery TEST_BATTERY_ONE = new Battery()
        .withID("1")
        .withManufacturer("MK")
        .withModel("ES17-12")
        .withBuildDate("2025-08-15")
        .withPurchaseDate("2026-01-15")
        .withRanking(3)
        .withUseCase(useCase.COMPETITION)
        .withRatedVoltage(12.8)
        .withRatedCapacity(18.0)
        .withInternalResistance(0.020);

    public static final Battery TEST_BATTERY_TWO = new Battery()
        .withID("2")
        .withManufacturer("MK")
        .withModel("ES17-12")
        .withBuildDate("2025-08-15")
        .withPurchaseDate("2026-02-05")
        .withRanking(2)
        .withUseCase(useCase.COMPETITION)
        .withRatedVoltage(12.8)
        .withRatedCapacity(18.0)
        .withInternalResistance(0.018);

    public static final Battery TEST_BATTERY_THREE = new Battery()
        .withID("3")
        .withManufacturer("Energizer")
        .withModel("EN18-12")
        .withBuildDate("2025-12-01")
        .withPurchaseDate("2026-02-05")
        .withRanking(1)
        .withUseCase(useCase.PRACTICE)
        .withRatedVoltage(12.8)
        .withRatedCapacity(18.0)
        .withInternalResistance(0.015);

    public static ArrayList<Battery> BATTERY_LIST = new ArrayList<>();

    static {
        BATTERY_LIST.add(TEST_BATTERY_ONE);
        BATTERY_LIST.add(TEST_BATTERY_TWO);
        BATTERY_LIST.add(TEST_BATTERY_THREE);
    }
    

}
