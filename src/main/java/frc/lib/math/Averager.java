package frc.lib.math;

import java.util.ArrayDeque;
import java.util.Deque;

public class Averager {
    private final Deque<Double> measurements = new ArrayDeque<>();
    private final int windowSize;
    private double derivative = 0;

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
        double prevValue = getValue();
        measurements.addLast(measurement);
        while(measurements.size() > windowSize) {
            measurements.removeFirst();
        }
        double currValue = getValue();
        derivative = Math.abs(prevValue - currValue);
    }

    public double getDerivative(){
        return derivative;
    }

    public boolean hasMeasurements() {
        return !measurements.isEmpty();
    }
}
