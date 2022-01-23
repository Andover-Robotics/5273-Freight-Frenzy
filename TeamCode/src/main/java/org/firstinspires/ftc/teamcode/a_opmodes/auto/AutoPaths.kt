package org.firstinspires.ftc.teamcode.a_opmodes.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.GlobalConfig
import org.firstinspires.ftc.teamcode.GlobalConfig.carousel
import org.firstinspires.ftc.teamcode.GlobalConfig.outtakeCube
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

    val bot: Bot = Bot.getInstance()
    val drive: RRMecanumDrive = bot.roadRunner
    val paths: MutableList<AutoPathElement> = mutableListOf();

    val Double.toRadians get() = (toRadians(this))
    val Int.toRadians get() = (this.toDouble().toRadians)

    private fun Pose2d.reverse() = copy(heading = heading + PI)
    private var lastPosition: Pose2d = Pose2d()

    private var redAlliance = (GlobalConfig.alliance == GlobalConfig.Alliance.RED)
    private var depotSide = (GlobalConfig.side == GlobalConfig.Side.DEPOT)

    private var multiplier = if (redAlliance) 1 else -1

    private var intakeDelay = 375
    private var carouselDelay = 1250
    private var reverseDelay = 500
    private var bucketDelay = 750

    val turnRadians =  - PI / 2


    fun makePath(name: String, trajectory: Trajectory): AutoPathElement.Path {
        lastPosition = trajectory.end()
        return AutoPathElement.Path(name, trajectory)
        //Start of list of trajectories should not be lastPosition
    }

    private val intakeStart = AutoPathElement.Action("start intake") {
        if (redAlliance) bot.intake.runRight() else bot.intake.runLeft()
    }
    private val intakeStop = AutoPathElement.Action("stop intake") {
        //while (if (redAlliance) ! bot.intake.wasIntakedRight() else ! bot.intake.wasIntakedLeft()) {Thread.sleep(intakeDelay.toLong())}
        Thread.sleep(intakeDelay.toLong())
        if (redAlliance) bot.intake.reverseRight() else bot.intake.reverseLeft()
        if (redAlliance) bot.outtake.closeRightFlap() else bot.outtake.closeLeftFlap()
        Thread.sleep(reverseDelay.toLong())
        bot.intake.stop()
    }

    private val outtakeHigh = AutoPathElement.Action("Outtake High") {bot.outtake.goToTopGoal()}

    private val outtakeMid = AutoPathElement.Action("Outtake Mid") {bot.outtake.goToMidGoal()}

    private val outtakeLow = AutoPathElement.Action("Outtake Low") {bot.outtake.goToLowGoal() }

    private val outtakeEnd = AutoPathElement.Action("Outtake End") {
        bot.outtake.autoRun()
        bot.outtake.flipBucket()
        Thread.sleep(bucketDelay.toLong())
        bot.outtake.unFlipBucket()
        bot.outtake.fullyRetract()
        bot.outtake.autoRun()
        if (redAlliance) bot.outtake.closeLeftFlap() else bot.outtake.closeRightFlap()
        if (redAlliance) bot.outtake.openRightFlap() else bot.outtake.openLeftFlap()
    }

    // Kotlin 1.3 does not support inline instantiation of SAM interfaces
    class MarkerCallbackImpl(val func: () -> Unit) : MarkerCallback {
        override fun onMarkerReached() = func()
    }

    //TODO: insert action vals here

    private val runCarousel = AutoPathElement.Action("Run carousel motor") {if (redAlliance) bot.carousel.runRed() else bot.carousel.runBlue() }

    private val carouselWait = AutoPathElement.Action("Wait for Carousel") {
        Thread.sleep(carouselDelay.toLong())
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
    //TODO: Insert pose/vector values here

    fun initialPosition(): Pose2d {
        return Pose2d(  multiplier * 65.0, (if (redAlliance)  -31.0 else -41.0) + (if (depotSide) 0.0 else 48.0),3 * PI / 2 - if (redAlliance) 0.0 else PI)
    }

    private fun carouselPosition(): Pose2d {
        return Pose2d( multiplier * (if (redAlliance) 61.0 else 59.0), (if (redAlliance) -65.0 else -63.0), 3 * PI / 2 - if (redAlliance) 0.0 else PI)
    }

    private fun initialOuttakePosition(): Pose2d {
        return Pose2d(multiplier * 40.0, if (depotSide) -25.0 else -1.0, (if (redAlliance) 1 else -1) * (5 * (PI / 4)))
    }

    private fun intakePosition(n: Int): Pose2d {
        var offset = 7.0
        return Pose2d( multiplier * 68.0, 51.0 + offset * ( n - 1),3 * PI / 2 - if (redAlliance) 0.0 else PI)
    }

    private fun intermediateWaypoint(): Pose2d {
        return Pose2d(12.0, multiplier * 64.0, 3 * PI / 2  - if (redAlliance) 0.0 else PI)
    }

    private fun outtakePosition(): Pose2d {
        return Pose2d(multiplier * 53.0,- 15.0, 3 * PI / 2  - if (redAlliance) 0.0 else PI)
    }

    private fun parkingPosition(): Pose2d {
        return if (depotSide) Pose2d( multiplier * 36.0, -60.0, multiplier * 3 * PI / 2) else Pose2d(multiplier * 78.0,56.0, 3 * PI / 2  - if (redAlliance) 0.0 else PI)
    }

    private fun outtakeTangents(): List<Double> {
        val blueAllianceStartTangent = 1 * PI / 4
        val redAllianceStartTangent = 7 * PI / 4
        val blueAllianceEndTangent = 3 * PI / 4
        val redAllianceEndTangent = 5 * PI / 4
        return listOf( if (redAlliance) redAllianceStartTangent else blueAllianceStartTangent - if (redAlliance) 0.0 else PI, if (redAlliance) redAllianceEndTangent else blueAllianceEndTangent - if (redAlliance) 0.0 else PI);
    }

    private fun intakeTangents(initialIntake: Boolean): List<Double> {
        val warehouseSideStartTangent = if (redAlliance) if (initialIntake) 2 * PI else 2 * PI else if (initialIntake) 2 * PI else 2 * PI
        val depotSideStartTangent =  if (redAlliance) if (initialIntake) PI / 2 else 2 * PI else if (initialIntake) 2 * PI else 2 * PI
        val warehouseSideEndTangent = if (redAlliance) if (initialIntake) 3 * PI / 4 else PI / 2 else if (initialIntake) 4 * PI / 4 else 3 * PI / 2
        val depotSideEndTangent =  if (redAlliance) if (initialIntake) PI / 2 else PI / 2 else if (initialIntake) PI / 2 else PI / 2
        return listOf((if (depotSide) depotSideStartTangent else warehouseSideStartTangent) - if (redAlliance) 0.0 else PI, if (depotSide) depotSideEndTangent else warehouseSideEndTangent - if (redAlliance) 0.0 else PI)
    }

    private fun parkingTangents(): List<Double> {
        val depotSideStartTangent = 7 * PI / 4
        val depotSideEndTangent = 3 * PI / 2
        val warehouseSideStartTangent = 0 * PI / 4
        val warehouseSideEndTangent = 3 * PI / 2

        return listOf(if (depotSide) depotSideStartTangent else warehouseSideStartTangent - if (redAlliance) 0.0 else PI, if (depotSide) depotSideEndTangent else warehouseSideEndTangent - if (redAlliance) 0.0 else PI)
    }

    private fun intakeTrajectory(n: Int, initialIntake: Boolean): AutoPathElement.Path {
        var startingPose = if (n > 1) intakePosition(n - 1)
                            else if (carousel) carouselPosition()
                            else if (outtakeCube) initialOuttakePosition()
                            else initialPosition()

        return makePath("Intake Trajectory #$n",
            bot.roadRunner.trajectoryBuilder(startingPose)
                .splineToLinearHeading(intakePosition(n), intakeTangents(initialIntake)[1])
                .build())
    }

    private fun outtakeTrajectory(n: Int): AutoPathElement.Path {
        return makePath("Following Outtake Trajectory",
            bot.roadRunner.trajectoryBuilder(intakePosition(n), outtakeTangents()[0])
                .splineToLinearHeading(outtakePosition(), outtakeTangents()[1])
                .build())
    }

    //TODO: Implement a flag system, like this position is for spline to spline and so on so forth

    //TODO: Implement a drift system

    /*
    private val path = listOf( // what if we have one big 3d array with all our paths, and add that to our calc paths func
        Pose2d(65.0, -34.0, 0.0.toRadians), // remove + 2 later for all
        Pose2d(44.0, -20.0, (-45.0).toRadians),
        outtakeHigh,
        carouselPosition,
        runCarousel,
        intakeStart,
        Pose2d(64.0,  50.0 + 8, 0.0.toRadians),
        intakeStop,
        Pose2d(50.0+3.0, -9.0, 0.0.toRadians),
        outtakeHigh,
        intakeStart,
        Pose2d(64.0+8,  50.0 + 15, 0.0.toRadians),
        intakeStop,
        Pose2d(50.0+12, -6.0, 0.0.toRadians),
        outtakeHigh,
        Pose2d(64.0+12,  46.0+10, 0.0.toRadians),
        prepare
    )

  */

    val initialOuttakeTrajectory = makePath("Outtake Preloaded Cube Trajectory",
        bot.roadRunner.trajectoryBuilder(initialPosition())
            .lineToSplineHeading(initialOuttakePosition())
            .build()
    )

    val carouselTrajectory = makePath("Carousel Trajectory",
        bot.roadRunner.trajectoryBuilder(initialOuttakePosition())
            .lineToSplineHeading(carouselPosition())
            .build())

    val turnTrajectory = AutoPathElement.Action("Turn Trajectory"){ bot.roadRunner.turn(- turnRadians) }
    val unTurnTrajectory = AutoPathElement.Action("Turn Trajectory"){ bot.roadRunner.turn( turnRadians) }

    /*

    val initialIntakeTrajectory = intakeTrajectory(1)

    val followingOuttakeTrajectory = outtakeTrajectory(1)

    val followingIntakeTrajectory = intakeTrajectory(2)

    val thirdOuttakeTrajectory = outtakeTrajectory(2)

     */

    // TODO: add depot parking
    val parkingTrajectory = makePath("Parking Trajectory",
        bot.roadRunner.trajectoryBuilder(outtakePosition(), parkingTangents()[0])
            .splineToSplineHeading(parkingPosition(), parkingTangents()[1])
            .build())

    /*
    val tangents = listOf(
        listOf<Double>((180.0).toRadians, (210.0).toRadians),
        listOf<Double>((45.0).toRadians, (270.0).toRadians),
        listOf<Double>((90.0).toRadians, (210.0).toRadians)
    )

     */

    /*
    val rightStartPose = path[0] as Pose2d
    val leftStartPose = path[0] as Pose2d

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

            else if (path[i+1] == carouselPosition){
                p += (makePath("Path", drive.trajectoryBuilder(path[prevPath] as Pose2d).lineToSplineHeading(path[i+1] as Pose2d).addSpatialMarker(Vector2d(44.0, -20.0)) { runCarousel }.build()))
            }

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
                    p += (makePath("Path", drive.trajectoryBuilder(path[prevPath] as Pose2d, Tangents()[tanCount][0]).splineToLinearHeading(path[i+1] as Pose2d, Tangents()[tanCount][1]).build()))
                prevPath = i + 1
                tanCount += 1
            }
            i += 1
        }
    }

     */

    private fun calcTrajectories(pipelineResult: DuckDetector.PipelineResult, outtakeCube: Boolean, carousel: Boolean, cycles: Int, parking: Boolean): List<AutoPathElement> {

        if (outtakeCube) {
            paths.add(initialOuttakeTrajectory)
            if (pipelineResult == DuckDetector.PipelineResult.LEFT)
                paths.add(outtakeHigh)
            else if (pipelineResult == DuckDetector.PipelineResult.MIDDLE)
                paths.add(outtakeMid)
            else
                paths.add(outtakeLow)
            paths.add(outtakeEnd)
        }

        if (carousel) {
            paths.add(runCarousel)
            paths.add(carouselTrajectory)
            if (!redAlliance)
                paths.add(turnTrajectory)
            paths.add(carouselWait)
            if (!redAlliance)
                paths.add(unTurnTrajectory)
        }

        for (n in IntRange(1, cycles)) {
            paths.add(intakeStart)
            paths.add(if ( n == 1) intakeTrajectory(n, true) else intakeTrajectory(n, false))
            paths.add(intakeStop)
            paths.add(outtakeTrajectory(n))
            paths.add(outtakeHigh)
            paths.add(outtakeEnd)
        }

        if (parking) {
            paths.add(parkingTrajectory)
        }

        return paths

    }

    //TODO: Implement Types of Autos
    // ====================================================

    /*

     private val trajectorySets: Map<DuckDetector.PipelineResult, List<AutoPathElement>> = mapOf(
        //use !! when accessing maps ie: dropSecondWobble[0]!!
        //example
        DuckDetector.PipelineResult.RIGHT to run {
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
            /*

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

             */

        },
        DuckDetector.PipelineResult.LEFT to run {
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
        })

     */


    // features as functions, that return command groups
    fun getTrajectories(a: DuckDetector.PipelineResult): List<AutoPathElement> {
        return calcTrajectories(a, GlobalConfig.outtakeCube, GlobalConfig.carousel, GlobalConfig.cycles, GlobalConfig.parking)
    }
}