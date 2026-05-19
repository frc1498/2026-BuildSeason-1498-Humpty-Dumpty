package frc.robot.util.Zone;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Zone.CompositeZone.Operation;

public class CircleZone implements Zone {

    private String name;
    private Translation2d center;
    private double radius;

    public CircleZone(String name, Translation2d center, double radius) {
        this.name = name;
        this.center = center;
        this.radius = radius;
    }

    @Override
    public boolean containsPoint(Translation2d point) {
        return point.getDistance(this.center) <= this.radius;
    }

    @Override
    public Zone union(Zone other) {
        return new CompositeZone(this.name, this, other, Operation.UNION);
    }

    @Override
    public Zone intersection(Zone other) {
        return new CompositeZone(this.name, this, other, Operation.INTERSECTION);
    }

    @Override
    public Zone difference(Zone other) {
        return new CompositeZone(this.name, this, other, Operation.DIFFERENCE);
    }

    @Override
    public Zone complement(Zone other) {
        return new CompositeZone(this.name, this, other, Operation.COMPLEMENT);
    }
}
