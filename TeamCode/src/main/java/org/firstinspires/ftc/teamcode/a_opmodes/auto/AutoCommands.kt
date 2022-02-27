package org.firstinspires.ftc.teamcode.a_opmodes.auto


import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.arcrobotics.ftclib.command.*
import org.firstinspires.ftc.teamcode.GlobalConfig
import org.firstinspires.ftc.teamcode.b_hardware.Bot
import org.firstinspires.ftc.teamcode.b_hardware.subsystems.Outtake
import kotlin.math.atan2
import java.lang.Math.toRadians

object AutoCommands {

    // remeber to use endPose and pos to make it easier please

    // wrap trajectories in a command
    // wrap outtake controls in commands (BucketUp, BucketDown)
    // wrap carousel controls in commands
    // wrap intake controls in commands
    // waiting should be a command

    val alliance = GlobalConfig.alliance
    val redAll = GlobalConfig.Alliance.RED
    val bluAll = GlobalConfig.Alliance.BLUE

    val isRed = alliance == redAll

    val Double.toRadians get() = (toRadians(this))

    val offset = -90.0.toRadians

    class FollowTrajectory(val bot: Bot, val trajectory: Trajectory) : CommandBase() {
        override fun initialize() = bot.roadRunner.followTrajectoryAsync(trajectory)
        override fun execute() = bot.roadRunner.update()
        override fun isFinished() = !bot.roadRunner.isBusy
    }

    fun generateRunCarouselFeature(drift: Vector2d = Vector2d()): CommandBase {
        val bot = Bot.getInstance()
        val pos = bot.roadRunner.poseEstimate
        val startTangent = if (pos.x > 56) 315.0.toRadians + offset else if (pos.x < -56) 135.0.toRadians + offset else pos.heading
        val traj = bot.roadRunner.trajectoryBuilder(pos, startTangent)
                .splineToLinearHeading(
                        if(isRed)
                            Pose2d(55.0, -58.0, 0.0.toRadians + offset) + Pose2d(drift, 0.0)
                        else
                            Pose2d(-55.0, -58.0, 0.0.toRadians + offset) + Pose2d(drift, 0.0),
                        if(isRed) 45.0.toRadians + offset
                        else -45.0.toRadians + offset)
        return SequentialCommandGroup(
                ParallelCommandGroup(
                        FollowTrajectory(bot, traj.build()),
                        InstantCommand(if (isRed) bot.carousel::runRed else bot.carousel::runBlue),
                        bot.outtake.RunSlides(Outtake.RETRACTED, Outtake.SlideState.RETRACTED)),
                WaitCommand(1800),
                InstantCommand(bot.carousel::stop))
    }

    fun generateExitWarehouseFeature(drift: Vector2d = Vector2d()): CommandBase {
        val bot = Bot.getInstance()
        val pos = bot.roadRunner.poseEstimate
        val endPos =
                if(isRed) Vector2d(64.0, 0.0) + drift
                else Vector2d(64.0, 0.0) + drift
        return ParallelCommandGroup(
            bot.outtake.RunSlides(Outtake.RETRACTED, Outtake.SlideState.RETRACTED),
            FollowTrajectory(bot, bot.roadRunner.trajectoryBuilder(pos,
                    if(isRed) 180.0.toRadians
                    else 0.0.toRadians)
                    .strafeTo(endPos).build())
        )
    }

    fun generateOuttakeFeature(drift:   Vector2d = Vector2d()): CommandBase {
        val bot = Bot.getInstance()
        val pos = bot.roadRunner.poseEstimate
        val endPos = if(isRed)
                     Pose2d(50.0, -12.0, 0.0.toRadians + offset) + Pose2d(drift, 0.0)
        else
                Pose2d(-50.0, -12.0, 180.0.toRadians + offset) + Pose2d(drift, 0.0)
        var startTangent = 30.0.toRadians + offset

        if (!bot.isInWarehouse) {
            startTangent = atan2((pos.y - endPos.y),(pos.x-endPos.x)).toRadians + offset
        }

        val traj = bot.roadRunner.trajectoryBuilder(pos, startTangent)
                .splineToLinearHeading(endPos,
                        if(isRed) 270.0.toRadians + offset
                        else 90.0.toRadians + offset)

        return SequentialCommandGroup(
                ParallelCommandGroup(
                        FollowTrajectory(bot, traj.build()),
                        SequentialCommandGroup(
                                WaitCommand(600),
                                InstantCommand(bot.intake::stop, bot.intake),
                                InstantCommand(
                                        if(isRed) bot.outtake::closeLeftFlap
                                        else bot.outtake::closeRightFlap, bot.outtake),
                                InstantCommand(
                                        if(isRed) bot.outtake::closeRightFlap
                                        else bot.outtake::closeLeftFlap, bot.outtake),
                                bot.outtake.RunSlides(Outtake.TOP_GOAL_POS,
                                        Outtake.SlideState.AT_TOP_GOAL))),
                InstantCommand(bot.outtake::flipBucket, bot.outtake),
                WaitCommand(1000),
                InstantCommand(bot.outtake::unFlipBucket, bot.outtake)
        )
    }
    fun generateIntakeFeature(drift: Vector2d = Vector2d()): CommandBase {
        val bot = Bot.getInstance()
        val pos = bot.roadRunner.poseEstimate
        val endPos =
                if(isRed) Pose2d(Vector2d(64.0, 58.0), 0.0 + offset) + Pose2d(drift, 0.0)
                else Pose2d(Vector2d(-64.0, 58.0), 180.0 + offset) + Pose2d(drift, 0.0)

        val startTangent = if(isRed) 90.0.toRadians + offset
                           else -90.0.toRadians + offset;

        val trajToBarrier = bot.roadRunner.trajectoryBuilder(pos, startTangent)
//            .lineToLinearHeading(Pose2d(64.0, 20.0, 0.0 + offset))
            .splineToLinearHeading(endPos,
                    if(isRed) 210.0.toRadians + offset
                    else 30.0.toRadians + offset)
            .build()
//        val trajToWarehouse = bot.roadRunner.trajectoryBuilder(trajToBarrier.end(), 180.0.toRadians)
//            .strafeTo(Vector2d(64.0, 58.0))
//            .build()

        return SequentialCommandGroup(
                InstantCommand(
                        if(isRed) bot.outtake::closeLeftFlap
                        else bot.outtake::closeRightFlap, bot.outtake),
                InstantCommand(
                        if(isRed) bot.outtake::openRightFlap
                        else bot.outtake::openLeftFlap, bot.outtake),
                InstantCommand(
                        if(isRed) bot.intake::runRight
                        else bot.intake::runLeft, bot.intake),
                ParallelDeadlineGroup(FollowTrajectory(bot, trajToBarrier),
                        SequentialCommandGroup(
                            bot.outtake.RunSlides(Outtake.RETRACTED, Outtake.SlideState.RETRACTED),
                            InstantCommand(
                                    if(isRed) bot.intake::runRight
                                    else bot.intake::runLeft, bot.intake),
                            InstantCommand(
                                    if(isRed) bot.intake::reverseLeft
                                    else bot.intake::reverseRight, bot.intake))),
                InstantCommand(
                        if(isRed) bot.intake::reverseRight
                        else bot.intake::reverseLeft, bot.intake),
                WaitCommand(500))
    }

    fun generateParkDepotFeature(drift: Vector2d = Vector2d()): CommandBase {
        val bot = Bot.getInstance()
        val endPos =
                if(isRed) Vector2d(39.0, -63.0) + drift
                else Vector2d(-39.0, -63.0) + drift
        val traj = bot.roadRunner.trajectoryBuilder(bot.roadRunner.poseEstimate)
            .strafeTo(endPos).build()

        return ParallelDeadlineGroup(FollowTrajectory(bot, traj),
            bot.outtake.RunSlides(Outtake.RETRACTED, Outtake.SlideState.RETRACTED))
    }

    fun generatePreloadedFreightFeature(drift: Vector2d = Vector2d()): CommandBase {
        val bot = Bot.getInstance()
        val pos = bot.roadRunner.poseEstimate
        val startTangent = if (pos.x > 56) 315.0.toRadians + offset else pos.heading
        val endPos =
                if(isRed) Pose2d(Vector2d(0.0, -24.0) + drift, (-45.0).toRadians + offset)
                else Pose2d(Vector2d(0.0, -24.0) + drift, (-135.0).toRadians + offset)
        val traj = bot.roadRunner.trajectoryBuilder(pos, startTangent)
                .lineToSplineHeading(Pose2d(Vector2d(.0, -24.0) + drift, (-45.0).toRadians + offset))

        return SequentialCommandGroup(
                bot.outtake.cmdCloseLeftFlap,
                bot.outtake.cmdCloseRightFlap,
                ParallelCommandGroup(
                        FollowTrajectory(bot, traj.build()),
                        SequentialCommandGroup(
                                WaitCommand(600),
                                bot.outtake.RunSlides(Outtake.TOP_GOAL_POS,
                                        Outtake.SlideState.AT_TOP_GOAL))),
                bot.outtake.cmdFlipBucket,
                WaitCommand(1000),
                bot.outtake.cmdUnflipBucket)
    }

    private val Bot.isInWarehouse get() =
        roadRunner.poseEstimate.x > 24 || roadRunner.poseEstimate.x < -24 && roadRunner.poseEstimate.y > 24
}