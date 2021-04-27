package org.firstinspires.ftc.vrhsrobotics.victorianvoltage.auto.math.spline;

public class Waypoint
{
    private double xVal, yVal;
    public Waypoint(double x, double y)
    {
        xVal = x;
        yVal = y;
    }
    public double getXcoord()
    {
        return xVal;
    }
    public double getYcoord()
    {
        return yVal;
    }
    public String toString()
    {
        return "x: " + xVal + ", y: " + yVal;
    }
}