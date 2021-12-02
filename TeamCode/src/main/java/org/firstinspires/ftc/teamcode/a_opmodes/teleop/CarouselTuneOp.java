package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

//TODO: make a opmode able to change the speed of the carousel wheel to tune what speed exactly we can do
// maybe increment in 1 percent increments, and remember that you can use any buttons
// maybe teach luke the FTClib stuff with this

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.d_util.utilclasses.BinarySearchHelper;

import java.util.ArrayDeque;
import java.util.Deque;

public class CarouselTuneOp extends BaseOpMode{


    // start from middle for a binary search to the perfect rpm to spin carousel at
    double MIN = 0, MID = 1150.0/2.0, MAX = 1150;

    BinarySearchHelper curIter = new BinarySearchHelper(MIN, MID, MAX);

    Deque<BinarySearchHelper> iters = new ArrayDeque<>();

    @Override
    void subInit() {
        iters.push(curIter);
    }

    @Override
    void subLoop() {
        bot.carousel.runAtRPM(curIter.getMid());
        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            iters.push(curIter.iterateLeft());
            curIter = curIter.iterateLeft();
            System.gc();
        }
        else if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            iters.push(curIter.iterateRight());
            curIter = curIter.iterateRight();
            System.gc();
        }

        telemetry.addData("Current Carousel RPM = ", curIter.getMid());
        telemetry.addData("Current iteration of search = ", iters.size());
        telemetry.update();

        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.B)) {
            curIter = iters.pop();
            System.gc();
        }
    }

    @Override
    public void stop() {
        iters = null;
        telemetry.addData("Final Rpm = ", curIter.getMid());
        telemetry.update();
        curIter = null;
        System.gc();
    }
}


