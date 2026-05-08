package frc.robot.util.Zone;

import edu.wpi.first.math.geometry.Translation2d;

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
}
