package frc.robot.util.Zone;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Zone.CompositeZone.Operation;

public class PolygonZone implements Zone {

    private String name;
    private Translation2d[] vertices;

    public PolygonZone(String name, Translation2d... vertices) {
        this.name = name;
        this.vertices = vertices;
    }

    @Override
    public boolean containsPoint(Translation2d point) {
        // The article 'Inclusion of a Point in a Polygon' by Dan Sunday goes into the complete algorithm.
        // https://web.archive.org/web/20130126163405/http://geomalgorithms.com/a03-_inclusion.html

        int windingNumber = 0;

        // Iterate through each point in the polygon.
        for(int i = 0; i < this.vertices.length; i++) {

            if (this.vertices[i].getY() <= point.getY()) { // Check if the polygon vertice is below or level with the point
                if (this.vertices[i+1].getY() > point.getY()) { // If the vertice directly after the current vertice is above the point, then the vector created between the two vertices upwardly crosses the ray created by the point.
                    if (this.isLeft(this.vertices[i], this.vertices[i+1], point) > 0) { // Check if the point is to the left of the vector created by the two vertices.
                        ++windingNumber; // If yes, the vector has an upwards intersect with the point.  Increment the winding number.
                    }
                }
            } else { // This assumes that the polygon vertice is above the point.
                if (this.vertices[i+1].getY() <= point.getY()) { // If the vertice directly after the current vertice is below the point, then the vector created between the two vertices downwardly crosses the ray created by the point.
                    if (this.isLeft(this.vertices[i], this.vertices[i+1], point) < 0) { // Check if the point is to the right of the vector created by the two vertices.  Because the vector is pointing downwards, this is an upside down left-side check.
                        --windingNumber; // If yes, the vector has a downwards intersect with the point.  Decrement the winding number.
                    }
                }
            }
        }

        // As long as the winding number is not zero, the point is within the polygon.
        return windingNumber != 0;
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

    /**
     * Determine the area of the triangle created by three points.
     * The sign of the area determines if point two is located to the left or right of the vector created between points zero and one.
     * @param pointZero - The first side of the triangle.  Used as the starting point of the two vectors that create the triangle.
     * @param pointOne - The second side of the triangle.  A vector is created between this point and point zero.
     * @param pointTwo - The third side of the triangle.  This is the point we want to check the orientation of.
     * @return The area of the triangle.  If >= 0, point two is above point zero and one.  If <= 0, point two is below point zero and one.  If == 0, point two is in line with point zero and one.
     */
    private double isLeft(Translation2d pointZero, Translation2d pointOne, Translation2d pointTwo) {
        // With three points of a triangle, the distance between points (between point zero and point one, and point zero to point two) are vectors.
        // By treating the triangle as a flat triangle on a 3D plane, the cross product can be taken.
        // Taking the cross product of these vectors gives the area of the triangle.
        // The sign of the area gives the orientation of point two in relation to the vector created between point zero and point one.
        // If the area is positive, point two is to the left (or above) the vector between point zero and point one.
        // If the area is negative, point two is to the right (or below) the vector between point zero and point one.

        // The article 'Area of Triangles and Polygons' by Dan Sunday goes into the complete algorithm.
        // https://web.archive.org/web/20130406084141/http://geomalgorithms.com/a01-_area.html
        return 
            ((pointOne.getX() - pointZero.getX()) * (pointTwo.getY() - pointZero.getY())) - 
            ((pointTwo.getX() - pointZero.getX()) * (pointOne.getY() - pointZero.getY()));
    }
}
