package org.firstinspires.ftc.teamcode.a_opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.GlobalConfig
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.DuckDetector
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive
import org.firstinspires.ftc.teamcode.b_hardware.Bot
import java.lang.Math.toRadians
import kotlin.math.PI

class AutoPaths(val opMode: LinearOpMode) {//TODO: possibly add the TeleOpPaths functionality to this

    //TODO: reverse this

    sealed class AutoPathElement(open val name: String) {
        class Path(override val name: String, val trajectory: Trajectory) : AutoPathElement(name)

        //AutoPathElement.Path(name, trajectory)
        class Action(override val name: String, val runner: () -> Unit) : AutoPathElement(name)
        //AutoPathElement.Action(name) {actions to take(include sleeps)}
    }


    private fun Pose2d(x: Double, result: Map<DuckDetector.PipelineResult, Double>, heading: Double): Pose2d {
        return Pose2d(x, mapOf(DuckDetector.PipelineResult.LEFT to -67.5, DuckDetector.PipelineResult.RIGHT to 67.5), 180.0)
    }

    val bot: Bot = Bot.getInstance()
    val drive: RRMecanumDrive = bot.roadRunner

    val Double.toRadians get() = (toRadians(this))
    val Int.toRadians get() = (this.toDouble().toRadians)
    private fun Pose2d.reverse() = copy(heading = heading + PI)
    private var lastPosition: Pose2d = Pose2d()
    private var redAlliance = (GlobalConfig.alliance == GlobalConfig.Alliance.RED)
    private var multiplier = if (redAlliance) 1 else -1

    fun makePath(name: String, trajectory: Trajectory): AutoPathElement.Path {
        lastPosition = trajectory.end()
        return AutoPathElement.Path(name, trajectory)
        //Start of list of trajectories should not be lastPosition
    }

    private val intakeStart = AutoPathElement.Action("start intake") {
        if (redAlliance) bot.intake.runRight() else bot.intake.runLeft()
    }
    private val intakeStop = AutoPathElement.Action("stop intake") {
        while (if (redAlliance) ! bot.intake.isRightIntaking else ! bot.intake.isLeftIntaking) {Thread.sleep(25)}
        if (redAlliance) bot.intake.reverseRight() else bot.intake.reverseLeft()
        if (redAlliance) bot.outtake.closeRightFlap() else bot.outtake.closeLeftFlap()
        //bot.intake.stop()
        Thread.sleep(1000)
        if (redAlliance) bot.intake.reverseRight() else bot.intake.reverseLeft()
        if (redAlliance) bot.outtake.closeRightFlap() else bot.outtake.closeLeftFlap()
    }

    private val outtakeHigh = AutoPathElement.Action("Outtake High") {
        bot.outtake.goToTopGoal()}

    private val outtakeMid = AutoPathElement.Action("Outtake Mid") {
        bot.outtake.goToMidGoal()}

    private val outtakeLow = AutoPathElement.Action("Outtake Low") {
        bot.outtake.goToLowGoal()
        }

    private val outtakeEnd = AutoPathElement.Action("Outtake End") {
        bot.outtake.autoRun()
        bot.outtake.flipBucket()
        Thread.sleep(1000)
        bot.outtake.unFlipBucket()
        bot.outtake.fullyRetract()
        bot.outtake.autoRun()
        if (redAlliance) bot.outtake.closeLeftFlap() else bot.outtake.closeRightFlap()
        if (redAlliance) bot.outtake.openRightFlap() else bot.outtake.openLeftFlap()
        if (redAlliance) bot.carousel.runRed() else bot.carousel.runBlue()
    }

    //Probably won't be used, but here just in case
    /*
    fun makeAction(name: String, action: Unit): AutoPathElement.Action{
        return AutoPathElement.Action(name, action)
        //Redundant but conforms to naming scheme
    }
*/
    // Kotlin 1.3 does not support inline instantiation of SAM interfaces
    class MarkerCallbackImpl(val func: () -> Unit) : MarkerCallback {
        override fun onMarkerReached() = func()
    }

    //TODO: insert action vals here

    private val runCarousel = AutoPathElement.Action("Run carousel motor") {
        if (redAlliance) bot.carousel.runRed() else bot.carousel.runBlue()
    }

    private val carouselWait = AutoPathElement.Action("Wait for Carousel") {
        Thread.sleep(1250)
        bot.carousel.stop()
    }

    private val prepare = AutoPathElement.Action("Prepare for teleOp") {
        if (redAlliance) bot.outtake.closeLeftFlap() else bot.outtake.closeRightFlap()
        if (redAlliance) bot.outtake.openRightFlap() else bot.outtake.openLeftFlap()
    }
    
    fun asVector2D(pos: Pose2d) : Vector2d
    {
        return Vector2d(pos.x, pos.y)
    }
    //TODO: Insert pose/vector vals here //
    val offset = -90.0.toRadians // need to subtract drift from the previos position to be accurate
    // implement a on off feature, so the driver can pick and choose what we do for the auto
    var xDrift = 0 // maybe implement a flag system, like this pos is for spline to spline ans so
    // on

    public val initialPosition = Pose2d(multiplier * 65.0, -31.0, 0.0.toRadians + offset)
    private val carouselPosition = Pose2d(multiplier * 61.0, -63.0, 0.0.toRadians + offset)
    private val intialOuttakeCubePosition = Pose2d(multiplier * 44.0, -20.0, (-45.0).toRadians + offset)
    private val initialIntakePosition = Pose2d(multiplier * 64.0,  54.0-4.0, 0.0.toRadians + offset)
    private val initialIntakeTangents = listOf((180.0).toRadians + offset, (210.0).toRadians + offset)
    private val followingOuttakePosition = Pose2d(multiplier * 57.0, -15.0-3.0, 0.0.toRadians + offset)
    private val followingOuttakeTangents = listOf((45.0).toRadians + offset, (270.0).toRadians + offset);
    private val followingIntakePosition = Pose2d(multiplier * 72.0,  61.0 - 6.0, 0.0.toRadians + offset)
    private val followingIntakeTangents = listOf((90.0).toRadians + offset, (210.0).toRadians + offset)
    private val thirdOuttakePosition = Pose2d(multiplier * 62.0, -6.0-3.0, 0.0.toRadians + offset)
    private val thirdOuttakeTangents = listOf((45.0).toRadians + offset, (270.0).toRadians + offset)
    private val parkingPosition = Pose2d(multiplier * 78.0, 56.0, 0.0.toRadians + offset)
    private val parkingTangents = listOf((90.0).toRadians + offset, (210.0).toRadians + offset)

    private val path = listOf( // what if we have one big 3d array with all our paths, and add that to our calc paths func
            Pose2d(65.0, -34.0, 0.0.toRadians + offset), // remove + 2 later for all
            Pose2d(44.0, -20.0, (-45.0).toRadians + offset),
            outtakeHigh,
            carouselPosition,
            runCarousel,
            intakeStart,
            Pose2d(64.0,  50.0 + 8, 0.0.toRadians + offset),
            intakeStop,
            Pose2d(50.0+3.0, -9.0, 0.0.toRadians + offset),
            outtakeHigh,
            intakeStart,
            Pose2d(64.0+8,  50.0 + 15, 0.0.toRadians + offset),
            intakeStop,
            Pose2d(50.0+12, -6.0, 0.0.toRadians + offset),
            outtakeHigh,
            Pose2d(64.0+12,  46.0+10, 0.0.toRadians + offset),
            prepare
            )

    val initialOuttakeTrajectory = makePath("Outtake Preloaded Cube Trajectory",
        bot.roadRunner.trajectoryBuilder(initialPosition)
            .lineToSplineHeading(intialOuttakeCubePosition)
            .build()
    )

    val carouselTrajectory = makePath("Carousel Trajectory",
        bot.roadRunner.trajectoryBuilder(lastPosition)
            .lineToSplineHeading(carouselPosition)
            .addSpatialMarker(Vector2d(lastPosition.x, lastPosition.y)){runCarousel}
            .build())

    val initialIntakeTrajectory = makePath("Initial Intake Trajectory",
        bot.roadRunner.trajectoryBuilder(lastPosition, initialIntakeTangents[0])
            .splineToLinearHeading(initialIntakePosition, initialIntakeTangents[1])
            .addSpatialMarker(Vector2d(lastPosition.x, lastPosition.y)) {intakeStart}
            .build())

    val followingOuttakeTrajectory = makePath("Following Outtake Trajectory",
        bot.roadRunner.trajectoryBuilder(lastPosition, followingOuttakeTangents[0])
            .splineToLinearHeading(followingOuttakePosition, followingOuttakeTangents[1])
            //.addSpatialMarker(Vector2d(lastPosition.x, lastPosition.y)){intakeStop}
            .build())

    val followingIntakeTrajectory = makePath("Following Intake Trajectory",
        bot.roadRunner.trajectoryBuilder(lastPosition, followingIntakeTangents[0])
            .splineToLinearHeading(followingIntakePosition, followingOuttakeTangents[1])
            .addSpatialMarker(Vector2d(lastPosition.x, lastPosition.y)){intakeStart}
            .build())

    val thirdOuttakeTrajectory = makePath("Third Outtake Trajectory",
        bot.roadRunner.trajectoryBuilder(lastPosition, thirdOuttakeTangents[0])
            .splineToLinearHeading(thirdOuttakePosition, thirdOuttakeTangents[1])
            //.addSpatialMarker(Vector2d(lastPosition.x, lastPosition.y)){intakeStop}
            .build())

    val parkingTrajectory = makePath("Parking Trajectory",
        bot.roadRunner.trajectoryBuilder(lastPosition, parkingTangents[0])
            .splineToLinearHeading(parkingPosition, parkingTangents[1])
            .addSpatialMarker(Vector2d(lastPosition.x, lastPosition.y)){prepare}
            .build())

    val tangents = listOf(
            //listOf<Double>((-60.0).toRadians + offset, (-45.0).toRadians + offset),
            //listOf<Double>((-45.0).toRadians + offset, (-45.0).toRadians + offset),
            listOf<Double>((180.0).toRadians + offset, (210.0).toRadians + offset),
            listOf<Double>((45.0).toRadians + offset, (270.0).toRadians + offset),
            listOf<Double>((90.0).toRadians + offset, (210.0).toRadians + offset),
            listOf<Double>((45.0).toRadians + offset, (270.0).toRadians + offset),
            listOf<Double>((90.0).toRadians + offset, (210.0).toRadians + offset)

    )
    val rightStartPose = path[0] as Pose2d
    val leftStartPose = path[0] as Pose2d

    //TODO: Make Trajectories in trajectorySets
    private fun calcTrajectories() : List<AutoPathElement>
    {
        val p = mutableListOf<AutoPathElement>()
        var i = 0
        var prevPath = 0
        var tanCount = 0
        while (true)
        {
            if (path[i+1] == prepare)
            {
                p += path[i+1] as AutoPathElement.Action
                return p
            }
/*
            else if (path[i+1] == carouselPosition){
                p += (makePath("Path", drive.trajectoryBuilder(path[prevPath] as Pose2d).lineToSplineHeading(path[i+1] as Pose2d).addSpatialMarker(Vector2d(44.0, -20.0)) { runCarousel }.build()))
            }
*/
            else if  (path[i+1] is AutoPathElement.Action)
            {
                p += path[i + 1] as AutoPathElement.Action
            }

            else if (path[i+1] is Int)
            {
                xDrift += path[i+1] as Int
            }

            else if (path[i+1] is Pose2d || path[i+1] is Vector2d) {
                if ((path[i+1] as Pose2d).heading != (path[prevPath] as Pose2d).heading)
                    p += (makePath("Path", drive.trajectoryBuilder(path[prevPath] as Pose2d).lineToSplineHeading(path[i+1] as Pose2d).build()))
                else
                    p += (makePath("Path", drive.trajectoryBuilder(path[prevPath] as Pose2d, tangents[tanCount][0]).splineToLinearHeading(path[i+1] as Pose2d, tangents[tanCount][1]).build()))
                prevPath = i + 1
                tanCount += 1
            }
            i += 1
        }
    }
    //TODO: Separate alliance-specific and position-specific paths                                                                            ====================================================
    private val trajectorySets: Map<DuckDetector.PipelineResult, List<AutoPathElement>> = mapOf(
            //use !! when accessing maps ie: dropSecondWobble[0]!!
            //example
            DuckDetector.PipelineResult.LEFT to run {
                //calcTrajectories()
                listOf(
                    initialOuttakeTrajectory,
                    outtakeLow,
                    outtakeEnd,
                    carouselTrajectory,
                    carouselWait,
                    intakeStart,
                    initialIntakeTrajectory,
                    intakeStop,
                    followingOuttakeTrajectory,
                    outtakeHigh,
                    outtakeEnd,
                    intakeStart,
                    followingIntakeTrajectory,
                    intakeStop,
                    thirdOuttakeTrajectory,
                    outtakeHigh,
                    outtakeEnd,
                    parkingTrajectory
                )
            },
            DuckDetector.PipelineResult.MIDDLE to run {
                //calcTrajectories()
                listOf(
                    initialOuttakeTrajectory,
                    outtakeMid,
                    outtakeEnd,
                    carouselTrajectory,
                    carouselWait,
                    intakeStart,
                    initialIntakeTrajectory,
                    intakeStop,
                    followingOuttakeTrajectory,
                    outtakeHigh,
                    outtakeEnd,
                    intakeStart,
                    followingIntakeTrajectory,
                    intakeStop,
                    thirdOuttakeTrajectory,
                    outtakeHigh,
                    outtakeEnd,
                    parkingTrajectory
                )
            },
            DuckDetector.PipelineResult.RIGHT to run {
                //calcTrajectories()
                listOf(
                    initialOuttakeTrajectory,
                    outtakeHigh,
                    outtakeEnd,
                    carouselTrajectory,
                    carouselWait,
                    intakeStart,
                    initialIntakeTrajectory,
                    intakeStop,
                    followingOuttakeTrajectory,
                    outtakeHigh,
                    outtakeEnd,
                    intakeStart,
                    followingIntakeTrajectory,
                    intakeStop,
                    thirdOuttakeTrajectory,
                    outtakeHigh,
                    outtakeEnd,
                    parkingTrajectory
                )
           }
    )
    // features as functions, that return command groups
    fun getTrajectories(a: DuckDetector.PipelineResult): List<AutoPathElement> {
        return trajectorySets[a]!!
    }
}