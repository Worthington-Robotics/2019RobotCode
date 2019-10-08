package frc.lib.geometry;

public interface ICurvature<S> extends State<S> {
    double getCurvature();

    /**
     *
     * @return change of curvature/change in arc length
     */
    double getDCurvatureDs();
}
