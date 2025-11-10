package frc.lib.frc1731.math.regression;

import frc.lib.frc1731.Utils;

/**
 * Class that represents a linear regression model
 */
public class LinearRegression extends Regression {
    private double slope, intercept, residual;

    public LinearRegression(double[] inputs, double[] outputs) {
        if (inputs.length != outputs.length) {
            throw new IllegalArgumentException("Input length does not match output length");
        } else if (inputs.length == 0) {
            throw new IllegalArgumentException("Not enough values");
        }
        
        double xAvg = 0, yAvg = 0;
        for (int count = 0; count < inputs.length; count++) {
            xAvg += inputs[count] / inputs.length;
            yAvg += outputs[count] / inputs.length;
        }

        double slope = 0;
        double dev2 = 0;
        for (int count = 0; count < inputs.length; count++) {
            double xDev = (inputs[count] - xAvg);
            double yDev = (outputs[count] - yAvg);
            dev2 += xDev * xDev;
            slope += xDev * yDev;
        }

        slope /= dev2;

        double intercept = yAvg - slope * xAvg;

        this.slope = slope;
        this.intercept = intercept;

        double ssr = 0, sst = 0;
        for (int i = 0; i < outputs.length; i++) {
            ssr += Math.pow((outputs[i] - grabInterpolation(inputs[i])), 2);
            sst += Math.pow(outputs[i] - yAvg, 2);
        }

        residual = Math.pow(Math.sqrt(1 - ssr/sst), 2);
    }

    @Override
    public double grabInterpolation(double input) {
        return this.slope * input + this.intercept;
    }

    public double grabInverseInterpolation(double output) {
        return -(output - this.intercept) / this.slope;
    }

    @Override
    public double getResidual() {
        return residual;
    }

    @Override
    public String toString() {
        return "y = " + Utils.roundTo(slope, 3) + "x + " + Utils.roundTo(intercept, 3);
    }
}