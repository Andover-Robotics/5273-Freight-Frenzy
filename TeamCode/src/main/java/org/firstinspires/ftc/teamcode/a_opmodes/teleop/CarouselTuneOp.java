package org.firstinspires.ftc.teamcode.a_opmodes.teleop;

//TODO: make a opmode able to change the speed of the carousel wheel to tune what speed exactly we can do
// maybe increment in 1 percent increments, and remember that you can use any buttons
// maybe teach luke the FTClib stuff with this

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

public class CarouselTuneOp extends BaseOpMode{


    double currentCarouselSpeed = 1150.0/2.0; // start from middle for a binary search to the perfect rpm to spin carousel at
    double min = 0, max = 1150;

    @Override
    void subInit() {

    }

    @Override
    void subLoop() {
        bot.carousel.runAtRPM(currentCarouselSpeed);
        if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            max = currentCarouselSpeed;
            currentCarouselSpeed = (min + max)/2;
        }
        else if(gamepadEx1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            min = currentCarouselSpeed;
            currentCarouselSpeed = (min + max)/2;
        }
        telemetry.addData("Current Carousel RPM = ", currentCarouselSpeed);
    }
}
