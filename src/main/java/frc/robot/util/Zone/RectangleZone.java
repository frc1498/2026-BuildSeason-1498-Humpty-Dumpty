package frc.robot.util.Zone;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Zone.CompositeZone.Operation;

public class RectangleZone implements Zone {

    private String name;
    private double xMinimum;
    private double xMaximum;
    private double yMinimum;
    private double yMaximum;
    
    public RectangleZone(String name, double xMinimum, double xMaximum, double yMinimum, double yMaximum) {
        this.name = name;
        this.xMinimum = xMinimum;
        this.xMaximum = xMaximum;
        this.yMinimum = yMinimum;
        this.yMaximum = yMaximum;
    }

    @Override
    public boolean containsPoint(Translation2d point) {
        return (point.getX() >= this.xMinimum) && (point.getX() <= this.xMaximum) && (point.getY() >= this.yMinimum) && (point.getY() <= this.yMaximum);
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
