package frc.robot.util;

import java.util.function.Supplier;

public class MercMath {
    public static Supplier<Double> zeroSupplier = () -> 0.0;

    public static double squareInput(double input) {
        return input > 0.0 ? Math.pow(input, 2) : -Math.pow(input, 2);
    }

    public static double RPMToMetersPerSecond(double rpm, double radius) {
        return (rpm * (2 * Math.PI * radius)) / 60.0; //TODO: fix placeholder number
    }
}
