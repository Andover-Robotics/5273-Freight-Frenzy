package org.firstinspires.ftc.teamcode.a_opmodes.auto

import android.util.Log
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.arcrobotics.ftclib.command.CommandBase
import com.arcrobotics.ftclib.command.CommandScheduler
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.a_opmodes.auto.AutoCommands.offset
import org.firstinspires.ftc.teamcode.b_hardware.Bot
import java.util.function.Supplier
import kotlin.reflect.jvm.reflect

@Autonomous(name = "Carousel All(Feature-based)", group = "Experimental")
class CarouselAllFeature : OpMode() {
    lateinit var bot: Bot
    lateinit var scheduler: CommandScheduler

    val features = listOf(
        // functions (don't call them yet!)
            "run carousel"  to { AutoCommands.generateRunCarouselFeature(Vector2d(1.0, -7.0)) },
            "deliver"       to { AutoCommands.generateOuttakeFeature(Vector2d(-3.0, -8.0)) },
            "intake"        to { AutoCommands.generateIntakeFeature(Vector2d(0.0, -8.0)) },
            "deliver cycle" to { AutoCommands.generateOuttakeFeature(Vector2d(2.0, 0.0)) },
            "intake"        to { AutoCommands.generateIntakeFeature(Vector2d(0.0, -8.0)) },
            "deliver cycle" to { AutoCommands.generateOuttakeFeature(Vector2d(2.0, 0.0)) },
            "park"          to { AutoCommands.generateParkDepotFeature(Vector2d(0.0, 0.0)) }
    )
    var currentFeature = 0

    override fun init() {
        currentFeature = 0
        Bot.instance = null
        bot = Bot.getInstance(this)
        bot.roadRunner.poseEstimate = Pose2d(65.0, -31.0, 0.0 + offset)
        scheduler = CommandScheduler.getInstance()

        scheduler.onCommandFinish {
            currentFeature += 1
            if (currentFeature >= features.size)
                requestOpModeStop()
            else {
                Log.i("FeatureControl", String.format("generating feature '%s'", features[currentFeature].first))
                scheduler.schedule(features[currentFeature].second())
            }
        }
    }

    override fun start() {
        scheduler.schedule(features[currentFeature].second())
    }

    override fun loop() {
        scheduler.run()

        if (currentFeature < features.size)
            telemetry.addData("current command", features[currentFeature].first)
    }
}