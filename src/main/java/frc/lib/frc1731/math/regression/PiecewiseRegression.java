package frc.lib.frc1731.math.regression;

/**
 * Class that represents a piecewise regression model
 */
public class PiecewiseRegression extends Regression {
    private double[] values;
    private double deltaDistance; // meters
    public PiecewiseRegression(double[] values, double deltaDistanceMeters) {
        if (values.length == 0) {
            throw new IllegalArgumentException("Input length does not match output length");
        }

        this.values = values;
        this.deltaDistance = deltaDistanceMeters;
    }

    @Override
    public double grabInterpolation(double input) {
        int startIndex = (int)(input/deltaDistance);
        int endIndex = startIndex + 1;

        if (startIndex >= values.length - 1) { // Above possible range
            return values[values.length - 1];
        } else if (startIndex < 0) { // Below possible range
            return values[0];
        }

        double remainder = (input - startIndex*deltaDistance);// / deltaDistance;
        double additional = remainder/deltaDistance*(values[endIndex] - values[startIndex]);

        return values[startIndex] + additional;
    }

    @Override
    public double getResidual() {
        return 1.0; // Can't necessarily have residuals
    }

    @Override
    public String toString() {
        return "Not a typical regression model";
    }
}