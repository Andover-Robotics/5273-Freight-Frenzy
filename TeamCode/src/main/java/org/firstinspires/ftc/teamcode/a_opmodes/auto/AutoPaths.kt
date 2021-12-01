package org.firstinspires.ftc.teamcode.a_opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.DuckDetector
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive
import org.firstinspires.ftc.teamcode.b_hardware.Bot
import java.lang.Math.toRadians
import kotlin.math.PI
import kotlin.math.roundToInt

class AutoPaths(val opMode: LinearOpMode) {//TODO: possibly add the TeleOpPaths functionality to this

    //TODO: reverse this

    sealed class AutoPathElement(open val name: String) {
        class Path(override val name: String, val trajectory: Trajectory): AutoPathElement(name)
        //AutoPathElement.Path(name, trajectory)
        class Action(override val name: String, val runner: () -> Unit): AutoPathElement(name)
        //AutoPathElement.Action(name) {actions to take(include sleeps)}
    }

    public val rightStartPose = Pose2d(- 12.0, - 66.5, 0.0)
    public val leftStartPose = Pose2d(- 12.0, 67.5, 0.0)


    private fun Pose2d(x: Double, result: Map<DuckDetector.PipelineResult, Double>, heading: Double): Pose2d {
        return Pose2d(x, mapOf(DuckDetector.PipelineResult.LEFT to - 67.5, DuckDetector.PipelineResult.RIGHT to 67.5), 180.0)
    }

    val bot: Bot = Bot.getInstance()
    val drive: RRMecanumDrive = bot.roadRunner
    val Double.toRadians get() = (toRadians(this))
    val Int.toRadians get() = (this.toDouble().toRadians)
    private fun Pose2d.reverse() = copy(heading = heading + PI)
    private var lastPosition: Pose2d = Pose2d()

    fun makePath(name: String, trajectory: Trajectory): AutoPathElement.Path{
        lastPosition = trajectory.end()
        return AutoPathElement.Path(name, trajectory)
        //Start of list of trajectories should not be lastPosition
    }

    //Probably won't be used, but here just in case
    fun makeAction(name: String, action: () -> Unit): AutoPathElement.Action{
        return AutoPathElement.Action(name, action)
        //Redundant but conforms to naming scheme
    }

    // Kotlin 1.3 does not support inline instantiation of SAM interfaces
    class MarkerCallbackImpl(val func: () -> Unit): MarkerCallback {
        override fun onMarkerReached() = func()
    }

    private fun turn(from: Double, to: Double): AutoPathElement.Action {
        return AutoPathElement.Action("Turn from ${Math.toDegrees(from).roundToInt()}deg" +
                "to ${Math.toDegrees(to).roundToInt()}deg") {
            bot.roadRunner.turn(to - from)
        }
    }


    //TODO: insert action vals here

    private val runCarousel = AutoPathElement.Action("Run carousel motor") {
              Thread.sleep(2000)
              bot.carousel.stop()
    }

    private val runSlides = AutoPathElement.Action("Run slides to shipping hubs"){
              bot.outtake.goToTopGoal()
              Thread.sleep(1500)
              bot.outtake.flipBucket()
              Thread.sleep(1000)
              bot.outtake.unFlipBucket()
              Thread.sleep(500)
              bot.outtake.fullyRetract()
    }
    //                                                                  =======================================================

    //example
    //private val shootRings = AutoPathElement.Action("Shoot 3 rings") {
    //        bot.shooter.shootRings(opMode, 3, 0.8)
    //        bot.shooter.turnOff()
    //        Thread.sleep(1000)
    //    }


    //Copy from here on into AutoPathVisualizer ==============================================================================

    //TODO: Insert pose/vector vals here

    private val strafe_distance = 28.0;

    // val elementY = (pipelineResult == TemplateDetector.PipelineResult.LEFT ? )

    //                                                                  ===================================================

    //example
    // private val dropSecondWobble = mapOf(
    //            0 to Pose2d(-4.2 + 1.5, -48.0 - 3.056 + 1f, (-90.0 + 30.268).toRadians),
    //            1 to Pose2d(24.0 - 9.45428 + 3f, -24.0 - 25.16465, (102.4 - 90.0).toRadians),
    //            4 to Pose2d(48 - 5.1, -48.0 - 3.0556 - 3f, (-90.0 + 30.268).toRadians)
    //    )

    //TODO: Make Trajectories in trajectorySets

    //TODO: Separate alliance-specific and position-specific paths                                                                            ====================================================
    private val trajectorySets: Map<DuckDetector.PipelineResult, List<AutoPathElement>> = mapOf(
            //use !! when accessing maps ie: dropSecondWobble[0]!!
            //example
            DuckDetector.PipelineResult.LEFT to run {
                listOf(
                    makePath("Strafe Right to Shipping Hub",
                        drive.trajectoryBuilder(leftStartPose)
                            .strafeRight(strafe_distance)
                            .build()),
                    runSlides,
                    makePath("Strafe Left to Wall",
                        drive.trajectoryBuilder(lastPosition)
                            .strafeLeft(strafe_distance - 2.0)
                            .build()),
                    makePath("Drive Backwards to Carousel",
                        drive.trajectoryBuilder(lastPosition)
                            .back(50.5)
                            .addSpatialMarker(Vector2d(-48.0, -66.5)){
//                                bot.carousel.run()
                            }
                            .build()),
                    runCarousel,
                    makePath("Drive Forward to Park",
                        drive.trajectoryBuilder(lastPosition)
                            .forward(102.0)
                            .build())
                    )
                    /* makePath("Pick Up Team Shipping Element",
                        drive.trajectoryBuilder(startPose.plus(Pose2d(-16.75, 0.0, 0.0)))
                            .splineToLinearHeading(elementPose, 100.0)
                            .build()))

                     */
            },

            DuckDetector.PipelineResult.RIGHT to run{
                listOf(
                    makePath("Strafe Right to Shipping Hub",
                        drive.trajectoryBuilder(rightStartPose)
                            .strafeRight(strafe_distance)
                            .build()),
                    runSlides,
                    makePath("Strafe Left to Wall",
                        drive.trajectoryBuilder(lastPosition)
                            .strafeLeft(strafe_distance - 2.0)
                            .build()),
                    makePath("Drive Backwards to Carousel",
                        drive.trajectoryBuilder(lastPosition)
                            .forward( 50.5)
                            .addSpatialMarker(Vector2d(-48.0, -66.5)){
                                bot.carousel.runRed()
                            }
                            .build()),
                    runCarousel,
                    makePath("Drive Forward to Park",
                        drive.trajectoryBuilder(lastPosition)
                            .back(96.0)
                            .build())
                )
            }
//
    )


    fun getTrajectories(a: DuckDetector.PipelineResult): List<AutoPathElement>{
        return trajectorySets[a]!!
    }


}