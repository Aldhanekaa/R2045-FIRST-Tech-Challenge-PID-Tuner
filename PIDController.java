
public class PIDController {
    public double kP = 0.1, kI = 0, kD = 0, errorTolerance = 0, iLimit = 0;

    public PIDController(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }

    public PIDController(double p, double i, double d, double tolerance, double ilimit) {
        kP = p;
        kI = i;
        kD = d;
        errorTolerance = tolerance;
        iLimit = ilimit;
    }
}
