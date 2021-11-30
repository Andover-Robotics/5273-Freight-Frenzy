package org.firstinspires.ftc.teamcode.b_hardware.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Carousel;

public class StartCarousel extends CommandBase {

    private Carousel carousel;

    public void StartCarousel(Carousel carousel){
        this.carousel = carousel;
        addRequirements(carousel);
    }

    @Override
    public void initialize(){
        carousel.run();
    }
}
