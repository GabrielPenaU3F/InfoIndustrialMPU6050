import org.apache.commons.math3.fitting.PolynomialCurveFitter;
import org.apache.commons.math3.fitting.WeightedObservedPoints;

import java.util.ArrayList;
import java.util.List;

public class RegressionCalculator {

    public double[] calculateRegressionCoefficients(double[] xData, double[] yData) {

        if (xData.length != yData.length) throw new RuntimeException("Incompatible data arrays");
        List<XYPoint> points = this.generateXYPoints(xData, yData);
        return this.calculateRegressionCoeficients(points);

    }

    private List<XYPoint> generateXYPoints(double[] xData, double[] yData) {
        List<XYPoint> points = new ArrayList<XYPoint>();
        for (int i=0; i < xData.length; i++) {
            points.add(new XYPoint(xData[i], yData[i]));
        }
        return points;
    }

    private double[] calculateRegressionCoeficients(List<XYPoint> data) {

        final WeightedObservedPoints observedPoints = new WeightedObservedPoints();
        for (XYPoint point : data) {
            observedPoints.add(point.getX(), point.getY());
        }

        final PolynomialCurveFitter fitter = PolynomialCurveFitter.create(5);
        final double[] coeff = fitter.fit(observedPoints.toList());
        return coeff;

    }

}
