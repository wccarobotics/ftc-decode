package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.LinkedList;

/**
 * Tracks loop timing statistics (average, min, max) over a rolling time window.
 */
public class LoopTimer {
    private final double windowSeconds;
    private final ElapsedTime timer = new ElapsedTime();
    private final LinkedList<Double> timestamps = new LinkedList<>();
    private final LinkedList<Double> loopTimes = new LinkedList<>();
    private double lastTime = -1;

    private double avgMs = 0;
    private double minMs = 0;
    private double maxMs = 0;

    public LoopTimer() {
        this(1.0);
    }

    public LoopTimer(double windowSeconds) {
        this.windowSeconds = windowSeconds;
    }

    /** Call once per loop iteration to record timing. */
    public void update() {
        double now = timer.seconds();
        if (lastTime >= 0) {
            double deltaMs = (now - lastTime) * 1000.0;
            timestamps.addLast(now);
            loopTimes.addLast(deltaMs);

            // Trim entries outside the window
            double cutoff = now - windowSeconds;
            while (!timestamps.isEmpty() && timestamps.peekFirst() < cutoff) {
                timestamps.removeFirst();
                loopTimes.removeFirst();
            }

            // Compute stats
            double sum = 0;
            double min = Double.MAX_VALUE;
            double max = Double.MIN_VALUE;
            for (double t : loopTimes) {
                sum += t;
                if (t < min) min = t;
                if (t > max) max = t;
            }
            avgMs = sum / loopTimes.size();
            minMs = min;
            maxMs = max;
        }
        lastTime = now;
    }

    public double getAvgMs()   { return avgMs; }
    public double getMinMs()   { return minMs; }
    public double getMaxMs()   { return maxMs; }
    public int getLoopCount()  { return loopTimes.size(); }
}
