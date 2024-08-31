
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private double kP, kI, kD;
    private double lastError = 0, integralSum = 0, reference = 0;
    private boolean isRunning = false;
    private double Setpoint;
    private ElapsedTime timer = new ElapsedTime();
    private double errorTolerance = 0, iLimit = 0;

    public PID(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;
    }
    public PID(double p, double i, double d, double ilimit) {
        kP = p;
        kI = i;
        kD = d;
        iLimit = ilimit;
    }
    public PID(PIDController pid) {
        kP = pid.kP;
        kI = pid.kI;
        kD = pid.kD;
        errorTolerance = pid.errorTolerance;
        iLimit = pid.iLimit;
    }

    public void setSetpoint(double setpoint) {
        Setpoint = setpoint;
        isRunning = false;
    }

    public double update(double position) {
        if (isRunning == false) {
            timer = new ElapsedTime();
        }

        // obtain the encoder position
        double encoderPosition = position;
        // calculate the error
        double error = reference - encoderPosition;

        double derivative = 0;
        if (errorTolerance != 0 && error < errorTolerance) {
            // rate of change of the error
            isRunning = false;
            return 0;
        }

        // sum of all error over time
        if (Math.abs(error) < iLimit) {
            integralSum += (error * timer.seconds());
        }

        double out = (kP * error) + (kI * integralSum) + (kD * derivative);

        lastError = error;
        return out;
    }

    public double getError() {
        return lastError;
    }
    public double getIntegralSum() {
        return integralSum;
    }
}
