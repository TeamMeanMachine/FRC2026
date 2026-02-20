package frc.team2471.frc2026

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.frc2026.tests.joystickTest
import frc.team2471.frc2026.tests.leftRightStaticFFTest
import frc.team2471.frc2026.tests.questOffsetTest
import frc.team2471.frc2026.tests.slipCurrentTest
import frc.team2471.frc2026.tests.velocityVoltTest
import frc.team2471.frc2026.tests.zeroTurretEncoders
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team2471.frc.lib.control.Autonomi


object Autonomous: Autonomi() {

//    val paths: MutableMap<String, Trajectory<SwerveSample>> = findChoreoPaths()  <-- already inside Autonomi

    /** Supplier that sets the robot's pose. Used inside [setDrivePositionToAutoStartPose] */
    override val drivePoseSetter: (Pose2d) -> Unit = { Drive.pose = it }

    /** Chooser for selecting autonomous commands */
    override val autoChooser: LoggedDashboardChooser<AutoCommand?> =
        LoggedDashboardChooser<AutoCommand?>("Auto Chooser").apply {
            addOption("8 Foot Straight", AutoCommand(eightFootStraight()))
            addOption("6x6 Square", AutoCommand(squarePathTest()))
        }

    /** Chooser for test commands */
    override val testChooser: LoggedDashboardChooser<Command?> =
        LoggedDashboardChooser<Command?>("Test Chooser").apply {
            // Set up SysId routines and test command options
            addOption("Drive Translation SysId ALL", Drive.sysIDTranslationAll())
            addOption("Drive Rotation SysId ALL", Drive.sysIDRotationAll())
            addOption("Drive Steer SysId ALL", Drive.sysIDSteerAll())
            addOption("Set Angle Offsets", Drive.setAngleOffsets())
            addOption("JoystickTest", joystickTest())
            addOption("Drive Slip Current Test", Drive.slipCurrentTest())
            addOption("Drive L/R Static FF Test", Drive.leftRightStaticFFTest())
            addOption("Drive Velocity Volt Test", Drive.velocityVoltTest())
            addOption("Quest offset Test", Drive.questOffsetTest())
            addOption("Turret Encoder Zero", zeroTurretEncoders())
        }

    /** Autonomous commands */

    private fun eightFootStraight(): Command {
        return Drive.driveAlongChoreoPath(paths["8 foot"]!!, resetOdometry = true)
    }

    private fun squarePathTest(): Command {
        return Drive.driveAlongChoreoPath(paths["square"]!!, resetOdometry = true)
    }
}