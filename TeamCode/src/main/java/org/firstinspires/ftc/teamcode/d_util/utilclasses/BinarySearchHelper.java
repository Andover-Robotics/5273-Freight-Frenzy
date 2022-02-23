package org.firstinspires.ftc.teamcode.d_util.utilclasses;

public class BinarySearchHelper {

    private double min, mid, max;

    public BinarySearchHelper(double min, double max) {
        this.min = min;
        this.mid = (min + max) / 2;
        this.max = max;
    }

    public BinarySearchHelper iterateRight() {
        return new BinarySearchHelper(mid, max);
    }
    public BinarySearchHelper iterateLeft() {
        return new BinarySearchHelper(min, mid);
    }

    public double getMax() {
        return max;
    }
    public double getMid() {
        return mid;
    }
    public double getMin() {
        return min;
    }
}
