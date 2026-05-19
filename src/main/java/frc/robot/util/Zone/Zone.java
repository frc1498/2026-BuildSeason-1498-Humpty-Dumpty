package frc.robot.util.Zone;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface Zone {

    boolean containsPoint(Translation2d point);

    default Trigger contains(Supplier<Translation2d> translation) {
            return new Trigger(() -> containsPoint(translation.get())); 
        }

    default Zone union(Zone other) {
        return null;
    }

    default Zone intersection(Zone other) {
        return null;
    }

    default Zone difference(Zone other) {
        return null;
    }

    default Zone complement(Zone other) {
        return null;
    }
}