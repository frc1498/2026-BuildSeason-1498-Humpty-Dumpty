package frc.robot.util.Zone;

import edu.wpi.first.math.geometry.Translation2d;

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
}
