package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;
import java.util.Map;

public class ColorSensorCache {

    private static final long MIN_INTERVAL_MS = 100;

    private boolean hasReadThisLoop = false;

    private static class CacheEntry {
        double distance = 0.0;
        long lastReadMs = 0;
    }

    private final Map<RevColorSensorV3, CacheEntry> cache = new HashMap<>();

    public void startLoop() {
        hasReadThisLoop = false;
    }

    public double getDistance(RevColorSensorV3 sensor, DistanceUnit unit) {
        if (sensor == null) return 0.0;

        long now = System.currentTimeMillis();
        CacheEntry entry = cache.get(sensor);
        if (entry == null) {
            entry = new CacheEntry();
            cache.put(sensor, entry);
        }

        boolean enoughTimePassed = (now - entry.lastReadMs) >= MIN_INTERVAL_MS;

        if (!hasReadThisLoop && enoughTimePassed) {
            try {
                entry.distance = sensor.getDistance(unit);
            } catch (Exception ignored) {
            }
            entry.lastReadMs = now;
            hasReadThisLoop = true;
        }

        return entry.distance;
    }

}
