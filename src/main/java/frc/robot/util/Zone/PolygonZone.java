package frc.robot.util.Zone;

import edu.wpi.first.math.geometry.Translation2d;

public class PolygonZone implements Zone {

    private String name;
    private Translation2d[] vertices;

    public PolygonZone(String name, Translation2d... vertices) {
        this.name = name;
        this.vertices = vertices;
    }

    @Override
    public boolean containsPoint(Translation2d point) {
        // Look at 'Inclusion of a Point in a Polygon' by Dan Sunday.
        // Also look at 'Area of Triangles and Polygons' by Dan Sunday.
        // From 'Point in polygon' Wikipedia page.
    }
}
