package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.spline;

public class Spline {
    private double[] xCords, yCords;
    private final int CURVE_POINTS = 3;
    private final double DEAD_WHEEL_TICKS_PER_REV = 4096;
    private double arcLen;


    public Spline(double[] xCords, double[] yCords) {
        this.xCords = xCords;
        this.yCords = yCords;
        arcLen =arcLength();
    }

    public double getXLength(double t)
    {
        double a = (Math.pow((1-t),3)*xCords[0]*DEAD_WHEEL_TICKS_PER_REV)+3*(Math.pow((1-t),2)*xCords[1]*DEAD_WHEEL_TICKS_PER_REV)+3*((1-t)*xCords[2]*DEAD_WHEEL_TICKS_PER_REV*Math.pow(t,2))+(Math.pow(t,3)*xCords[3]*DEAD_WHEEL_TICKS_PER_REV);
        System.out.println("x = "+a+" @t ="+t);
        return a;
    }


    public double getYLength(double t)
    {
        double a =(Math.pow((1-t),3)*yCords[0]*DEAD_WHEEL_TICKS_PER_REV)+3*(Math.pow((1-t),2)*yCords[1]*DEAD_WHEEL_TICKS_PER_REV)+3*((1-t)*yCords[2]*DEAD_WHEEL_TICKS_PER_REV*Math.pow(t,2))+(Math.pow(t,3)*yCords[3]*DEAD_WHEEL_TICKS_PER_REV);
        System.out.println("y = "+a+" @t ="+t);
        return a;
    }


    public double getDx(double t)
    {
        return 3*Math.pow((1-t),2)*(xCords[1]-xCords[0])*DEAD_WHEEL_TICKS_PER_REV+6*(1-t)*t*(xCords[2]-xCords[1])*DEAD_WHEEL_TICKS_PER_REV+3*Math.pow(t,2)*(xCords[3]-xCords[2])*DEAD_WHEEL_TICKS_PER_REV;
    }

    public double getDy(double t)
    {
        return 3*Math.pow((1-t),2)*(yCords[1]-yCords[0])*DEAD_WHEEL_TICKS_PER_REV+6*(1-t)*t*(yCords[2]-yCords[1])*DEAD_WHEEL_TICKS_PER_REV+3*Math.pow(t,2)*(yCords[3]-yCords[2])*DEAD_WHEEL_TICKS_PER_REV;

    }

    /**
     * arc length formula is ∫√((dx)^2+(dy)^2)dt
     * getDx is the dertive of getXLength
     * getDy is the dertive of getYLength
     * @return
     */
    private double arcLength()
    {
        double length = 0;
        for (double i = 0; i <=1; i+=0.05) {
            System.out.println("i = "+i);
           length +=  Math.sqrt(Math.pow(getDx(i),2)+Math.pow(getDy(i),2))*0.05;
           System.out.println(length);
        }

        return length;
    }

    public double getArcLen() {
        return arcLen;
    }

    @Override
    public String toString() {
        return ""+arcLength();
    }


}
