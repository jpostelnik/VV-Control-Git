package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.controltheory;

import org.ejml.simple.SimpleMatrix;

public class Kinematics {
    private double width, height;

    private SimpleMatrix J, R;

    public Kinematics(double width, double height) {
        this.width = width;
        this.height = height;

        J = new SimpleMatrix(new double[][]{
                {-1, 1, (width + height)},
                {1, 1, -(width + height)},
                {-1, 1, -(width + height)},
                {1, 1, (width + height)}
        });
    }

//    public SimpleMatrix getPower(double Vx, double Vy, double dTheta) {
//        SimpleMatrix cx = new SimpleMatrix(new double[][]{{Vy, Vx, dTheta}});
//        SimpleMatrix power = J.mult(R).mult(cx);
//        double maxPower = power.elementMaxAbs();
//
//        return power.scale(1 / maxPower);
//    }

    /**
     * Compute power to apply to each wheel relative to the robot
     * coordinate plane (y is forward, x is side-to-side)
     *
     * @param Vx     - component of velocity in x direction
     * @param Vy     - component of velocity in y direction
     * @param dTheta - rate of rotation around center of the robot
     * @return matrix of wheel angular velocities (power)
     */
    public SimpleMatrix getWheelPower(double Vx, double Vy, double dTheta) {
        SimpleMatrix cx = new SimpleMatrix(new double[][]{{Vx}, {Vy}, {dTheta}});
        SimpleMatrix power = J.mult(cx);
        double maxPower = power.elementMaxAbs();

        System.out.println("wheel power = "+ power.scale(1 / maxPower).transpose());

        return power.scale(1 / maxPower);
    }

    public void setOrientation(double theta) {
        theta = Math.toRadians(theta);
        this.R = new SimpleMatrix(new double[][]{
                {Math.cos(theta), Math.sin(theta), 0},
                {-Math.sin(theta), Math.cos(theta), 0},
                {0, 0, 1}
        });
    }

}
