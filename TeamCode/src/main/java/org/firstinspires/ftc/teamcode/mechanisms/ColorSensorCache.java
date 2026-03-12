package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.HashMap;
import java.util.Map;

public class ColorSensorCache {

    private static final long MIN_INTERVAL_MS = 100;

    private boolean hasReadThisLoop = false;

    private enum ValueType { DISTANCE, NORMALIZED }

    private static class Key {
        final RevColorSensorV3 sensor;
        final ValueType type;

        Key(RevColorSensorV3 sensor, ValueType type) {
            this.sensor = sensor;
            this.type = type;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (!(o instanceof Key)) return false;
            Key k = (Key) o;
            return this.sensor == k.sensor && this.type == k.type;
        }

        @Override
        public int hashCode() {
            return System.identityHashCode(sensor) * 31 + type.ordinal();
        }
    }

    private static class CacheEntry {
        Object value = null; // Double for distance (CM), NormalizedRGBA for normalized
        long lastReadMs = 0;
    }

    private final Map<Key, CacheEntry> cache = new HashMap<>();

    public void startLoop() {
        hasReadThisLoop = false;
    }

    public double getDistance(RevColorSensorV3 sensor, DistanceUnit unit) {
        if (sensor == null) return 0.0;

        long now = System.currentTimeMillis();
        Key distKey = new Key(sensor, ValueType.DISTANCE);
        Key normKey = new Key(sensor, ValueType.NORMALIZED);

        CacheEntry distEntry = cache.get(distKey);
        CacheEntry normEntry = cache.get(normKey);

        if (distEntry == null) {
            distEntry = new CacheEntry();
            cache.put(distKey, distEntry);
        }
        if (normEntry == null) {
            normEntry = new CacheEntry();
            cache.put(normKey, normEntry);
        }

        long distLastRead = distEntry.lastReadMs;
        boolean enoughTimePassed = (now - distLastRead) >= MIN_INTERVAL_MS;

        if (!hasReadThisLoop && enoughTimePassed) {
            try {
                double d = sensor.getDistance(DistanceUnit.CM);

                // populate distance entry only (distance always in CM)
                distEntry.value = d;
                distEntry.lastReadMs = now;
            } catch (Exception ignored) {
            }
            hasReadThisLoop = true;
        }

        Double cached = distEntry.value instanceof Double ? (Double) distEntry.value : null;
        return cached == null ? 0.0 : cached;
    }

    public NormalizedRGBA getNormalizedColors(RevColorSensorV3 sensor) {
        if (sensor == null) return null;

        long now = System.currentTimeMillis();
        Key normKey = new Key(sensor, ValueType.NORMALIZED);
        Key distKey = new Key(sensor, ValueType.DISTANCE);

        CacheEntry normEntry = cache.get(normKey);
        CacheEntry distEntry = cache.get(distKey);

        if (normEntry == null) {
            normEntry = new CacheEntry();
            cache.put(normKey, normEntry);
        }
        if (distEntry == null) {
            distEntry = new CacheEntry();
            cache.put(distKey, distEntry);
        }

        long normLastRead = normEntry.lastReadMs;
        boolean enoughTimePassed = (now - normLastRead) >= MIN_INTERVAL_MS;

        if (!hasReadThisLoop && enoughTimePassed) {
            try {
                NormalizedRGBA n = sensor.getNormalizedColors();
                NormalizedRGBA copy = new NormalizedRGBA();
                copy.red = n.red;
                copy.green = n.green;
                copy.blue = n.blue;
                copy.alpha = n.alpha;

                // populate normalized entry only
                normEntry.value = copy;
                normEntry.lastReadMs = now;
            } catch (Exception ignored) {
            }
            hasReadThisLoop = true;
        }

        return normEntry.value instanceof NormalizedRGBA ? (NormalizedRGBA) normEntry.value : null;
    }

    

}
