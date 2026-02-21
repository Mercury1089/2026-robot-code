package frc.robot.util;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class MercMath {
    public static Supplier<Double> zeroSupplier = () -> 0.0;

    public static double squareInput(double input) {
        return input > 0.0 ? Math.pow(input, 2) : -Math.pow(input, 2);
    }

    public static double RPMToMetersPerSecond(double rpm, double radius) {
        return (rpm * (2 * Math.PI * radius)) / 60.0;
    }

    public static double metersPerSecondToRPM(double mps, double radius) {
        return (mps * 60.0) / (2 * Math.PI * radius);
    }

    public static List<Double> removeOutliersIQR(List<Double> data) {
        if (data.isEmpty()) {
            return new ArrayList<>();
        }

        // Sort the data to find quartiles
        List<Double> sortedData = new ArrayList<>(data);
        Collections.sort(sortedData);

        int size = sortedData.size();
        
        // Calculate Q1 and Q3
        // For simplicity, we find the median of the lower and upper halves
        double q1 = getMedian(sortedData, 0, size / 2);
        double q3 = getMedian(sortedData, (size + 1) / 2, size);

        double iqr = q3 - q1;
        // The common multiplier for the threshold is 1.5
        double lowerBound = q1 - 1.5 * iqr;
        double upperBound = q3 + 1.5 * iqr;

        // Filter out outliers
        List<Double> filteredData = new ArrayList<>();
        for (Double value : data) {
            if (value >= lowerBound && value <= upperBound) {
                filteredData.add(value);
            }
        }
        return filteredData;
    }

    // Helper method to calculate the median of a sublist
    private static double getMedian(List<Double> data, int start, int end) {
        int subListSize = end - start;
        if (subListSize % 2 == 0) {
            // Even size
            int midIndex = start + subListSize / 2;
            return (data.get(midIndex - 1) + data.get(midIndex)) / 2.0;
        } else {
            // Odd size
            return data.get(start + subListSize / 2);
        }
    }
}
