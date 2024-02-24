package math;

import java.util.ArrayDeque;
import java.util.Deque;

public class Averager {
    private final Deque<Double> measurements = new ArrayDeque<>();
    private final int windowSize;

    public Averager(int windowSize) {
        this.windowSize = windowSize;
    }

    public void clear() {
        measurements.clear();
    }

    public double getValue() {
        double sum = 0;
        for(double measurement : measurements) {
            sum += measurement;
        }
        return sum / measurements.size();
    }

    public void addMeasurement(double measurement) {
        measurements.addLast(measurement);
        while(measurements.size() > windowSize) {
            measurements.removeFirst();
        }
    }

    public boolean hasMeasurements() {
        return !measurements.isEmpty();
    }
}
