package frc.lib.frc1731.math.regression;

/**
 * Class that represents a regression model
 */
public abstract class Regression {
    /**
     * Returns y assuming input is x
     */
    public abstract double grabInterpolation(double input);

    /**
     * Returns x assuming output is y
     */
    public double grabInverseInterpolation(double output) {
        return 0.0;
    }

    /**
     * R^2 value for the given regression
     */
    public abstract double getResidual();

    /**
     * Displays regression as a string
     */
    public abstract String toString();

    public enum RegressionType {
        kLinear,
        kQuadratic;
    }

    /**
     * Predicts the regression type based on residuals
     */
    public static RegressionType getRegressionType(double[] inputs, double[] outputs) {
        if (inputs.length != outputs.length) {
            throw new IllegalArgumentException("Input length does not match output length");
        } else if (inputs.length == 0) {
            throw new IllegalArgumentException("Not enough values");
        }

        Regression currentRegression = new LinearRegression(inputs, outputs); // Linear
        RegressionType predictedRegression = RegressionType.kLinear;
        double residual = currentRegression.getResidual();
        if (residual == 1) { // Definitive match
            return predictedRegression;
        }
        
        currentRegression = new QuadraticRegression(inputs, outputs); // Quadratic
        if (residual < currentRegression.getResidual()) {
            residual = currentRegression.getResidual();
            predictedRegression = RegressionType.kQuadratic;
        }

        return predictedRegression;
    }

    /**
     * Grabs the residual using the predictive 
     */
    public static double getResidual(double[] inputs, double[] outputs) {
        if (inputs.length != outputs.length) {
            throw new IllegalArgumentException("Input length does not match output length");
        } else if (inputs.length == 0) {
            throw new IllegalArgumentException("Not enough values");
        }

        switch (getRegressionType(inputs, outputs)) {
            case kLinear:
                return new LinearRegression(inputs, outputs).getResidual();
            case kQuadratic:
                return new QuadraticRegression(inputs, outputs).getResidual();
            default:
                return -1;
        }
    }

    /**
     * Grabs the value based on predicted regression type
     */
    public static double get(double input, double[] inputs, double[] outputs) {
        if (inputs.length != outputs.length) {
            throw new IllegalArgumentException("Input length does not match output length");
        } else if (inputs.length == 0) {
            throw new IllegalArgumentException("Not enough values");
        }

        switch (getRegressionType(inputs, outputs)) {
            case kLinear:
                return new LinearRegression(inputs, outputs).grabInterpolation(input);
            case kQuadratic:
                return new QuadraticRegression(inputs, outputs).grabInterpolation(input);
            default:
                return Double.NaN;
        }
    }
}