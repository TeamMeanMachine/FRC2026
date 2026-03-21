package frc.team2471.frc2026

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import frc.team2471.frc2026.tests.*
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team2471.frc.lib.control.Autonomi
import org.team2471.frc.lib.control.commands.parallelCommand
import org.team2471.frc.lib.control.commands.runCommand
import org.team2471.frc.lib.control.commands.runOnceCommand
import org.team2471.frc.lib.control.commands.sequenceCommand
import org.team2471.frc.lib.control.commands.waitUntilCommand
import org.team2471.frc.lib.swerve.sideToSideFlip


object Autonomous: Autonomi() {

//    val paths: MutableMap<String, Trajectory<SwerveSample>> = findChoreoPaths()  <-- already inside Autonomi

    /** Supplier that sets the robot's pose. Used inside [setDrivePositionToAutoStartPose] */
    override val drivePoseSetter: (Pose2d) -> Unit = { Drive.pose = it }

    /** Chooser for selecting autonomous commands */
    override val autoChooser: LoggedDashboardChooser<AutoCommand?> =
        LoggedDashboardChooser<AutoCommand?>("Auto Chooser").apply {
            addOption("8 Foot Straight", AutoCommand(eightFootStraight()))
            addOption("6x6 Square", AutoCommand(squarePathTest()))
            addOption("Double Swipe Left", AutoCommand(doubleSwipe(false), { paths["LeftSideDoubleSwipe"]!!.sideToSideFlip(false).getInitialPose(Drive.flipChoreoPaths).get() }))
            addOption("Double Swipe Right", AutoCommand(doubleSwipe(true), { paths["LeftSideDoubleSwipe"]!!.sideToSideFlip(true).getInitialPose(Drive.flipChoreoPaths).get() }))
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
            addOption("Full System Test", fullSystemTest())
            addOption("Turret Test", turretTest())
            addOption("Spindexer Test", spindexerTest())
            addOption("Shooter Test", shooterTest())
            addOption("Intake Deploy Test", intakeDeployTest())
            addOption("Intake Roller Test", intakeRollerTest())
        }

    /** Autonomous commands */

    private fun eightFootStraight(): Command {
        return Drive.driveAlongChoreoPath(paths["eightFoot"]!!, resetOdometry = true)
    }

    private fun squarePathTest(): Command {
        return Drive.driveAlongChoreoPath(paths["square"]!!, resetOdometry = true)
    }

    private fun doubleSwipe(doSideToSideFlip: Boolean): Command {
        val path = paths["LeftSideDoubleSwipe"]!!.sideToSideFlip(doSideToSideFlip)
        return parallelCommand(
            sequenceCommand(
                parallelCommand(
                    sequenceCommand(
                        Drive.driveAlongChoreoPath(path.getSplit(0).get(), resetOdometry = false),
                        Drive.driveAlongChoreoPath(path.getSplit(1).get(), resetOdometry = false, poseSupplier = Drive.localizer::pose),
                        runOnceCommand {
                            Intake.intakeState = Intake.IntakeState.OFF
                        }
                    ).withName("first driving double swipe auto"),
                    sequenceCommand(
                        waitUntilCommand { Intake.finishedHoming },
                        runOnceCommand {
                            Intake.deploy()
                            Intake.intakeState = Intake.IntakeState.INTAKING
                            println("Intake finished homing. Running Intake")
                        }
                    ).withName("Intake homing"),
                ).withName("First component Double swipe auto"),
                parallelCommand(
                    Shooter.shoot(true),
                    Intake.pulse()
                ).withTimeout(3.5),
                parallelCommand(
                    runOnceCommand {
                        Intake.deploy()
                        Intake.intakeState = Intake.IntakeState.INTAKING
                                   },
                    Drive.driveAlongChoreoPath(path.getSplit(2).get(), resetOdometry = false, poseSupplier = Drive.localizer::pose),
                    ),
                parallelCommand(
                    Shooter.shoot(true),
                    sequenceCommand(
                        runOnceCommand {
                            Intake.intakeState = Intake.IntakeState.OFF
                        },
                        Intake.pulse()
                    ),
//                    Drive.driveAlongChoreoPath(path.getSplit(3).get(), resetOdometry = false, poseSupplier = Drive.localizer::pose),
                )
            ).withName("Double Swipe auto sequence"),
            runCommand {
                Shooter.rampUpLoop()
            }.withName("Shooter ramp up loop")
        ).withName("Double swipe auto")
    }
}