package frc.robot.util.Zone;

import edu.wpi.first.math.geometry.Translation2d;

public class CompositeZone implements Zone {
    
    private String name;
    private Zone zoneA;
    private Zone zoneB;
    private Operation operation;

    public enum Operation {
        UNION,
        INTERSECTION,
        DIFFERENCE,
        COMPLEMENT
    }

    public CompositeZone(String name, Zone zoneA, Zone zoneB, Operation operation) {
        this.name = name;
        this.zoneA = zoneA;
        this.zoneB = zoneB;
        this.operation = operation;
    }

    @Override
    public boolean containsPoint(Translation2d point) {
        switch (this.operation) {
            case UNION:
                return this.zoneA.containsPoint(point) || this.zoneB.containsPoint(point);
            case INTERSECTION:
                return this.zoneA.containsPoint(point) && this.zoneB.containsPoint(point);
            case DIFFERENCE:
                return this.zoneA.containsPoint(point) && !this.zoneB.containsPoint(point);
            case COMPLEMENT:
                return !this.zoneA.containsPoint(point);
            default:
                return false;
        }
    }
}
