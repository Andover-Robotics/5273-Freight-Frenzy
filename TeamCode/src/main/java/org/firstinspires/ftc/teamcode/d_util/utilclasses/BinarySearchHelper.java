package org.firstinspires.ftc.teamcode.d_util.utilclasses;

public class BinarySearchHelper {

    private double min, mid, max;

    public BinarySearchHelper(double min, double mid, double max) {
        this.min = min;
        this.mid = mid;
        this.max = max;
    }

    public BinarySearchHelper iterateRight() {
        return new BinarySearchHelper(mid, (mid + min) / 2, max);
    }
    public BinarySearchHelper iterateLeft() {
        return new BinarySearchHelper(min, (min + mid) / 2, mid);
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
