package org.firstinspires.ftc.teamcode.a_opmodes.auto

import com.acmerobotics.roadrunner.followers.PathFollower
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.GlobalConfig
import org.firstinspires.ftc.teamcode.GlobalConfig.*
import org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline.DuckDetector
import org.firstinspires.ftc.teamcode.c_drive.RRMecanumDrive
import org.firstinspires.ftc.teamcode.b_hardware.Bot
import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Outtake
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

    private val intakeDelay = 1
    private val carouselDelay = 1750
    private val reverseDelay = 500
    private val bucketDelay = 600

    val turnRadians =  - 3 * PI / 4


    fun makePath(name: String, trajectory: Trajectory): AutoPathElement.Path {
        lastPosition = trajectory.end()
        return AutoPathElement.Path(name, trajectory)
        //Start of list of trajectories should not be lastPosition
    }

    private val intakeStart = AutoPathElement.Action("start intake") {
        if (redAlliance) bot.intake.runRight() else bot.intake.runLeft()
    }

    private val intaking = AutoPathElement.Action("stop intake") {
        bot.roadRunner.mode = RRMecanumDrive.Mode.IDLE
        bot.roadRunner.setWeightedDrivePower(Pose2d((if (redAlliance) - 1 else 1) * 0.5, 0.0, 0.0))
        while (if (redAlliance) ! bot.intake.wasIntakedRight() else ! bot.intake.wasIntakedLeft() && ! bot.outtake.isFreightIn()) {}
        bot.roadRunner.setWeightedDrivePower(Pose2d(0.0, 0.0, 0.0))
        bot.roadRunner.mode = RRMecanumDrive.Mode.FOLLOW_TRAJECTORY
        while (! bot.outtake.isFreightIn()) {}
        if (redAlliance) bot.outtake.closeRightFlap() else bot.outtake.closeLeftFlap()
        if (redAlliance) bot.intake.reverseRight() else bot.intake.reverseLeft()
        //Thread.sleep(500)
        if (redAlliance) if (Math.abs(bot.outtake.rightFlap.position - Outtake.FLAP_CLOSED) < 0.07) else bot.outtake.openRightFlap()
        else (if (Math.abs(bot.outtake.leftFlap.position - Outtake.FLAP_CLOSED) < 0.05) else bot.outtake.openLeftFlap())
    }

    private val intakeStop = AutoPathElement.Action("stop intake") {
        bot.intake.stop()
    }

    private val outtakeHigh = AutoPathElement.Action("Outtake High") {
        bot.outtake.goToTopGoal()
        bot.outtake.autoRun()
    }

    private val outtakeMid = AutoPathElement.Action("Outtake Mid") {
        bot.outtake.goToMidGoal()
        bot.outtake.autoRun()
    }

    private val outtakeLow = AutoPathElement.Action("Outtake Low") {
        bot.outtake.goToLowGoal()
        bot.outtake.autoRun()
    }

    private val outtakeEnd = AutoPathElement.Action("Outtake End") {
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

    //TODO: Edit
    fun initialPosition(): Pose2d {
        val initialY = (if (redAlliance)  -31.0 else -41.0)
        return Pose2d(multiplier * 65.0, initialY + (if (depotSide) 0.0 else 48.0),3 * PI / 2 - if (redAlliance) 0.0 else PI)
    }

    private fun carouselPosition(): Pose2d {
        return Pose2d( multiplier * (if (redAlliance) 61.0 else 59.0), (if (redAlliance) -62.5 else -67.0), 3 * PI / 2 - if (redAlliance) 0.0 else PI)
    }

    private fun initialOuttakePosition(): Pose2d {
        return Pose2d(multiplier * 40.0, if (depotSide) -22.0 else 0.0, (5 * (PI / 4)) - (if (redAlliance) 3 * PI / 2 else PI) - (if (depotSide) (if (redAlliance) PI / 2 else 3 * PI / 2) else 0.0))
    }

    //X: 72.0
    private fun intakePosition(n: Int): Pose2d {
        var offset = 3.0
        return Pose2d( multiplier * 72.0, 41.0 + offset * ( n - 1),3 * PI / 2 - if (redAlliance) 0.0 else PI)
    }

    private fun intermediateWaypoint(): Pose2d {
        return Pose2d(multiplier * 72.0, 12.0, 3 * PI / 2  - if (redAlliance) 0.0 else PI)
    }

    private fun outtakePosition(): Pose2d {
        return Pose2d(multiplier * 45.0,- 11.0, 3 * PI / 2  - if (redAlliance) 0.0 else PI)
    }

    private fun parkingPosition(): Pose2d {
        return if (depotSide) Pose2d( multiplier * 34.0, -66.0, 3 * PI / 2 - if (redAlliance) 0.0 else PI) else Pose2d(72.0,56.0, 3 * PI / 2  - if (redAlliance) 0.0 else PI)
    }

    private fun outtakeTangents(): List<Double> {
        val startTangent = PI / 2
        val endTangent = PI
        return listOf(startTangent - if (redAlliance) 0.0 else PI, endTangent - if (redAlliance) 0.0 else PI);
    }

    private fun intakeTangents(initialIntake: Boolean): List<Double> {
        val startTangent = 2 * PI
        val endTangent =  PI / 2
        return listOf(startTangent - if (redAlliance) 0.0 else (if (depotSide) 12 * PI / 8 else PI), endTangent /*- if (redAlliance) 0.0 else PI*/)
    }

    private fun parkingTangents(): List<Double> {
        val startTangent = if (depotSide) 3 * PI / 2 else 2 * PI
        val endTangent = if (depotSide) 3 * PI / 2 else PI / 2

        return listOf(startTangent - if (redAlliance) 0.0 else PI, endTangent - if (redAlliance) 0.0 else PI)
    }

    private fun intakeTrajectory(n: Int, initialIntake: Boolean): AutoPathElement.Path {

        var startingPose = if (carousel && n == 1)
            carouselPosition()
        else if (outtakeCube && n == 1)
            initialOuttakePosition().plus(Pose2d(0.0, 0.0, (if (redAlliance) -1 else 1) * PI / 4))
        else if (n == 1)
            initialPosition()
        else
            outtakePosition()

        return AutoPathElement.Path("Intake $n", bot.roadRunner.trajectoryBuilder(startingPose, intakeTangents(initialIntake)[0])
            .splineToLinearHeading(intermediateWaypoint(), intakeTangents(initialIntake)[1])
            .addTemporalMarker(0.01, intakeStart.runner)
            .lineToConstantHeading(asVector2D(intakePosition(n)))
            .build())
    }

    private fun outtakeTrajectory(n: Int): AutoPathElement.Path {
        return AutoPathElement.Path("Outtake $n", bot.roadRunner.trajectoryBuilder(bot.roadRunner.poseEstimate)
            .addTemporalMarker(0.5, outtakeHigh.runner)
            .lineToConstantHeading(asVector2D(intermediateWaypoint()))
            .splineToLinearHeading(outtakePosition(), outtakeTangents()[1])
            .build())
    }

    private fun parkingTrajectory(): AutoPathElement.Path {
        var parkingTrajectory = bot.roadRunner.trajectoryBuilder(if (cycles < 1 && !carousel) initialOuttakePosition() else if (carousel) carouselPosition() else outtakePosition(), parkingTangents()[0])
                .splineToLinearHeading(parkingPosition(), parkingTangents()[1])
                .build()

        if (cycles < 1 && !carousel and !outtakeCube)
            parkingTrajectory = bot.roadRunner.trajectoryBuilder(initialPosition(), intakeTangents(false)[0])
                .splineToLinearHeading(intermediateWaypoint(), intakeTangents(false)[1])
                .lineToConstantHeading(asVector2D(intakePosition(0)))
                .strafeRight(30.0)
                .build()

        return if (depotSide) AutoPathElement.Path("Parking", parkingTrajectory) else intakeTrajectory(2, false)
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
            .addTemporalMarker(0.1, outtakeHigh.runner)
            .lineToSplineHeading(initialOuttakePosition())
            .build()
    )

    val carouselTrajectory = makePath("Carousel Trajectory",
        bot.roadRunner.trajectoryBuilder(initialOuttakePosition())
            .lineToSplineHeading(carouselPosition())
            .addTemporalMarker(0.01, runCarousel.runner)
            .build())

    val turnTrajectory = AutoPathElement.Action("Turn Trajectory"){ bot.roadRunner.turn(- turnRadians)}
    val unTurnTrajectory = AutoPathElement.Action("Turn Trajectory"){ bot.roadRunner.turn(turnRadians)}

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
            paths.add(outtakeEnd)
        }
        //TODO: change initial position
        if (carousel) {
            //paths.add(runCarousel)
            paths.add(carouselTrajectory)
            if (!redAlliance)
                paths.add(turnTrajectory)
            paths.add(carouselWait)
            if (!redAlliance)
                paths.add(unTurnTrajectory)
        }

        for (n in IntRange(1, cycles)) {
            //paths.add(intakeStart)
            paths.add(if ( n == 1) intakeTrajectory(n, true) else intakeTrajectory(n, false))
            paths.add(intaking)
            paths.add(outtakeTrajectory(n))
            paths.add(intakeStop)
            //paths.add(outtakeHigh)
            paths.add(outtakeEnd)
        }

        if (parking) {
            paths.add(parkingTrajectory())
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