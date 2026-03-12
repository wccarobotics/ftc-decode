package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

/**
 * Caches color sensor reads and refreshes one entry per update() call in
 * round-robin order. Each sensor has separate entries for distance and color,
 * so they are read independently. Sensors are auto-registered on first access.
 */
public class ColorSensorCache {

    private enum ReadType { DISTANCE, COLOR }

    private static class CacheEntry {
        final RevColorSensorV3 sensor;
        final ReadType type;
        double distanceCm = 0.0;
        NormalizedRGBA colors = new NormalizedRGBA();
        long lastReadMs = 0;

        CacheEntry(RevColorSensorV3 sensor, ReadType type) {
            this.sensor = sensor;
            this.type = type;
        }

        void read() {
            if (type == ReadType.DISTANCE) {
                distanceCm = sensor.getDistance(DistanceUnit.CM);
            } else {
                NormalizedRGBA n = sensor.getNormalizedColors();
                colors.red = n.red;
                colors.green = n.green;
                colors.blue = n.blue;
                colors.alpha = n.alpha;
            }
            lastReadMs = System.currentTimeMillis();
        }
    }

    private final List<CacheEntry> entries = new ArrayList<>();
    private int nextIndex = 0;

    private CacheEntry findOrCreate(RevColorSensorV3 sensor, ReadType type) {
        for (CacheEntry entry : entries) {
            if (entry.sensor == sensor && entry.type == type) return entry;
        }
        CacheEntry entry = new CacheEntry(sensor, type);
        // First access: do an immediate read so callers get a real value
        entry.read();
        entries.add(entry);
        return entry;
    }

    /** Call once per loop. Reads one entry from the round-robin rotation. */
    public void update() {
        if (entries.isEmpty()) return;

        CacheEntry entry = entries.get(nextIndex);
        entry.read();
        nextIndex = (nextIndex + 1) % entries.size();
    }

    /** Get cached distance for a sensor. Auto-registers on first call. */
    public double getDistance(RevColorSensorV3 sensor, DistanceUnit unit) {
        if (sensor == null) return 0.0;
        return findOrCreate(sensor, ReadType.DISTANCE).distanceCm;
    }

    /** Get cached normalized colors for a sensor. Auto-registers on first call. */
    public NormalizedRGBA getNormalizedColors(RevColorSensorV3 sensor) {
        if (sensor == null) return null;
        return findOrCreate(sensor, ReadType.COLOR).colors;
    }

    /** Number of entries in the round-robin rotation. */
    public int getEntryCount() {
        return entries.size();
    }

    /** Age in ms of the least-recently-updated entry, or 0 if empty. */
    public long getMaxStalenessMs() {
        if (entries.isEmpty()) return 0;
        return System.currentTimeMillis() - entries.get(nextIndex).lastReadMs;
    }
}
