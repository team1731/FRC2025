package frc.lib.frc1731.math.regression;

import frc.lib.frc1731.Utils;

/**
 * Class that represents a quadratic regression model
 */
public class QuadraticRegression extends Regression {
    private double a, b, c, residual;
    public QuadraticRegression(double[] inputs, double[] outputs) {
        if (inputs.length != outputs.length) {
            throw new IllegalArgumentException("Input length does not match output length");
        } else if (inputs.length == 0) {
            throw new IllegalArgumentException("Not enough values");
        }

        // Calculating constants for y = ax^2 + bx + c
        double length = inputs.length;

        double sum_x = 0;
        double sum_y = 0;
        double sum_x2 = 0;
        double sum_x3 = 0;
        double sum_x4 = 0;
        double sum_xy = 0;
        double sum_x2y = 0;

        // Step 1: Calculate necessary summations
        for (int i = 0; i < length; i++) {
            sum_x += inputs[i];
            sum_y += outputs[i];
            sum_x2 += inputs[i]*inputs[i];
            sum_x3 += inputs[i]*inputs[i]*inputs[i];
            sum_x4 += inputs[i]*inputs[i]*inputs[i]*inputs[i];
            sum_xy += inputs[i]*outputs[i];
            sum_x2y += inputs[i]*inputs[i]*outputs[i];
        }

        // Step 2: Substitute into formulas
        double xx = sum_x2 - sum_x*sum_x/length;
        double xy = sum_xy - (sum_x*sum_y)/length;
        double xx2 = sum_x3 - (sum_x2*sum_x)/length;
        double x2y = sum_x2y - (sum_x2*sum_y)/length;
        double x2x2 = sum_x4 - (sum_x2*sum_x2)/length;

        // Step 3: Calculate constants
        double a = ((x2y*xx) - (xy*xx2)) / ((xx*x2x2) - (xx2*xx2));
        double b = ((xy*x2x2) - (x2y*xx2)) / ((xx*x2x2) - (xx2*xx2));
        double c = (sum_y/length) - (b*sum_x/length) - (a*sum_x2/length);

        this.a = a;
        this.b = b;
        this.c = c;

        // Step 4: Calculate residuals
        double ssr = 0, sst = 0;
        for (int i = 0; i < length; i++) {
            ssr += Math.pow((outputs[i] - grabInterpolation(inputs[i])), 2);
            sst += Math.pow(outputs[i] - sum_y/length, 2);
        }

        residual = Math.pow(Math.sqrt(1 - ssr/sst), 2);
    }

    @Override
    public double grabInterpolation(double input) {
        return a*(input*input)+b*input+c;
    }

    @Override
    public double getResidual() {
        return residual;
    }

    @Override
    public String toString() {
        return "y = " + Utils.roundTo(a, 3) + 
            "x^2 + " + Utils.roundTo(b, 3) + 
            "x + " + Utils.roundTo(c, 3) + 
            " (R^2 = " + Utils.roundTo(residual, 3) + ")";
    }
}