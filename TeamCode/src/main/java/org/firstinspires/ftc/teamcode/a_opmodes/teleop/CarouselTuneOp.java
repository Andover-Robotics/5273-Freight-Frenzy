package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

//TODO: make a opmode able to change the speed of the carousel wheel to tune what speed exactly we can do
// maybe increment in 1 percent increments, and remember that you can use any buttons
// maybe teach luke the FTClib stuff with this

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.d_util.utilclasses.BinarySearchHelper;

import java.util.ArrayDeque;
import java.util.Deque;

@TeleOp(name = "CarouselTuningOp", group = "Tuning")
public class CarouselTuneOp extends BaseOpMode{


    // start from middle for a binary search to the perfect rpm to spin carousel at
    double MOTOR_SPEED = 1150;
    double MIN = 0, MID = MOTOR_SPEED/2.0, MAX = 1150;


    BinarySearchHelper curIter = new BinarySearchHelper(MIN, MAX);

    Deque<BinarySearchHelper> iters = new ArrayDeque<>();

    @Override
    void subInit() {
        iters.push(curIter);
    }

    @Override
    void subLoop() {
        bot.carousel.runAtRPM(curIter.getMid());
        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            BinarySearchHelper nextIter = curIter.iterateLeft();
            iters.push(nextIter);
            curIter = nextIter;
        }
        else if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            BinarySearchHelper nextIter = curIter.iterateRight();
            iters.push(nextIter);
            curIter = nextIter;
        }

        telemetry.addData("Current Carousel RPM = ", curIter.getMid());
        telemetry.addData("Current iteration of search = ", iters.size());
        telemetry.update();

        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.B) && iters.size() > 0) {
            curIter = iters.pop();
        }
    }

    @Override
    public void stop() {
        iters = null;
        telemetry.addData("Final Rpm = ", curIter.getMid());
        telemetry.update();
        curIter = null;
    }
}


