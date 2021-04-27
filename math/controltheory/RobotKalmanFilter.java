package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory;

import org.ejml.simple.SimpleMatrix;

public class RobotKalmanFilter {

    private final double TICKS_PER_REV = 560;
    private final double DRIVETRAIN_WHEEL_DIAMTER = 4;
    private final double DRIVETRAIN_GEAR_RATIO = 1;
    private final double LINEAR_TO_TICKS = TICKS_PER_REV / (Math.PI * DRIVETRAIN_WHEEL_DIAMTER * DRIVETRAIN_GEAR_RATIO);
    private final double radius = 6.5;
    private final double STRAFE_COEFFICIENT = 1.20;
    private final double DEAD_WHEEL_DIAMTER = 2;
    private final double DEAD_WHEEL_TICKS_PER_REV = 4096;
    private final double DEAD_WHEEL_TO_TICKS = DEAD_WHEEL_TICKS_PER_REV / (Math.PI * DEAD_WHEEL_DIAMTER);

    //KalmanFilter Filter
    //updates ever 50 miliseconds
    private double dt = 0.025;
    private SimpleMatrix F = new SimpleMatrix(new double[][]{
            {1, 0, dt, 0},
            {0, 1, 0, dt},
            {0, 0, 1, 0},
            {0, 0, 0, 1}
    });

    private SimpleMatrix G = new SimpleMatrix(new double[][]{
            {0.5 * dt * dt},
            {0.5 * dt * dt},
            {dt},
            {dt}
    });

    private double sigma_a = 0.005;
    private SimpleMatrix Q = G.mult(G.transpose()).scale(sigma_a * sigma_a);

    private SimpleMatrix H = SimpleMatrix.identity(F.numRows());

    private double positionVar = 0.005;
    private double velocityVar = 0.001;

    private SimpleMatrix R = new SimpleMatrix(new double[][]{
            {positionVar, 0, 0, 0},
            {0, positionVar, 0, 0},
            {0, 0, velocityVar, 0},
            {0, 0, 0, velocityVar}
    });


    private double a_x = 20;
    private double a_y = 20;

    private SimpleMatrix u_u = new SimpleMatrix(new double[][]{
            {a_x},
            {a_y},
            {a_x},
            {a_y}
    });

    private SimpleMatrix u_0 = new SimpleMatrix(new double[][]{
            {0},
            {0},
            {0},
            {0}
    });

    private KalmanFilter robotKalmanFilter = new KalmanFilter(F, G, R, Q, H);

    private SimpleMatrix z;

    public RobotKalmanFilter(SimpleMatrix z) {
        this.z = z;
    }

    public void setZ(SimpleMatrix z) {
        this.z = z;
    }

    private SimpleMatrix estimateControlInput(double speed, int steadyStateSpeed) {
        SimpleMatrix u;
        if (speed < steadyStateSpeed) {
            u = u_u;
        } else {
            u = u_0;
        }
        return u;
    }


    /**
     * @param x    this is the x degrees/distance
     * @param xHat this is where i want to be
     * @return returns the error between this
     */
    public static double getError(double x, double xHat) {
        double e1 = xHat - x;
        double e2;
        if (x < 0) {
            e2 = xHat - (x + 360);
        } else {
            e2 = xHat - (x - 360);
        }
        if (Math.abs(e1) <= Math.abs(e2)) {
            return e1;
        }
        return e2;
    }


}
