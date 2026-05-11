package frc.team2471.frc2026

//import frc.team2471.frc2026.tests.*
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team2471.frc.lib.control.Autonomi
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.meters
import org.wpilib.command3.Command
import org.wpilib.math.geometry.Pose2d
import org.wpilib.smartdashboard.SendableChooser
import kotlin.math.absoluteValue


object Autonomous: Autonomi() {

//    val paths: MutableMap<String, Trajectory<SwerveSample>> = findChoreoPaths()  <-- already inside Autonomi

    /** Supplier that sets the robot's pose. Used inside [setDrivePositionToAutoStartPose] */
    override val drivePoseSetter: (Pose2d) -> Unit = { Drive.pose = it }

    /** Warmup function for speeding up auto loop times. Runs when selected auto changes. */
    override val warmupFunction: () -> Unit = {
        println("scheduling auto warmup")
//        Robot.commandScheduler.schedule(warmupDriveAlongPath())
        println("finished scheduling auto warmup")
    }

    /** Chooser for selecting autonomous commands */
    override val autoChooser: SendableChooser<AutoCommand> =
        SendableChooser<AutoCommand>().apply {
//            addOption("8 Foot Straight", AutoCommand(eightFootStraight()))
//            addOption("6x6 Square", AutoCommand(squarePathTest()))
//            addOption("Double Swipe Left", AutoCommand(doubleSwipe(false), { paths["LeftSideDoubleSwipe"]!!.sideToSideFlip(false).getInitialPose(Drive.flipChoreoPaths).get() }))
//            addOption("Double Swipe Right", AutoCommand(doubleSwipe(true), { paths["LeftSideDoubleSwipe"]!!.sideToSideFlip(true).getInitialPose(Drive.flipChoreoPaths).get() }))
//            addOption("Just Shoot", AutoCommand(justShoot()))
        }

    /** Chooser for test commands */
    override val testChooser: SendableChooser<Command> =
        SendableChooser<Command>().apply {
            // Set up SysId routines and test command options
//            addOption("Drive Translation SysId ALL", Drive.sysIDTranslationAll())
//            addOption("Drive Rotation SysId ALL", Drive.sysIDRotationAll())
//            addOption("Drive Steer SysId ALL", Drive.sysIDSteerAll())
            addOption("Set Angle Offsets", Drive.setAngleOffsets())
//            addOption("JoystickTest", joystickTest())
//            addOption("Drive Slip Current Test", Drive.slipCurrentTest())
//            addOption("Drive L/R Static FF Test", Drive.leftRightStaticFFTest())
//            addOption("Drive Velocity Volt Test", Drive.velocityVoltTest())
//            addOption("Quest offset Test", Drive.questOffsetTest())
//            addOption("Turret Encoder Zero", zeroTurretEncoders())
//            addOption("Shooter Curve Tuning", shooterCurveTuning())
//            addOption("Full System Test", fullSystemTest())
//            addOption("Turret Test", turretTest())
//            addOption("Spindexer Test", spindexerTest())
//            addOption("Shooter Test", shooterTest())
//            addOption("Intake Deploy Test", intakeDeployTest())
//            addOption("Intake Roller Test", intakeRollerTest())
        }

    /** Autonomous commands */

//    private fun eightFootStraight(): Command {
//        return Drive.driveAlongChoreoPath(paths["eightFoot"]!!, resetOdometry = true)
//    }
//
//    private fun squarePathTest(): Command {
//        return Drive.driveAlongChoreoPath(paths["square"]!!, resetOdometry = true)
//    }

//    private fun doubleSwipe(doSideToSideFlip: Boolean): Command {
//        val path = paths["LeftSideDoubleSwipe"]!!.sideToSideFlip(doSideToSideFlip)
//        var pathPercentage = 0.0
//        return parallelCommand(
//            sequenceCommand(
//                parallelCommand(
//                    Drive.driveAlongChoreoPath(path.getSplit(0).get(), resetOdometry = false),
//                    runOnceCommand {
//                        Turret.lookForwardOverride = true
//                    }
//                ),
//                parallelCommand(
//                    Drive.driveAlongChoreoPath(path.getSplit(1).get(), resetOdometry = false, poseSupplier = Drive.localizer::pose, exitSupplier = { percent, error -> pathPercentage = percent; Turret.lookForwardOverride = percent < 0.75; percent >= 1.0 }),
//                    sequenceCommand(
//                        waitUntilCommand { pathPercentage > 0.8 },
//                        runOnceCommand {
//                            Spindexer.disableReversingAuto = true
//                        },
//                        waitUntilCommand { pathPercentage > 0.9 },
//                        parallelCommand(
//                            Shooter.shoot(true),
//                            sequenceCommand(
//                                runOnceCommand {
//                                    Intake.intakeState = Intake.IntakeState.OFF
//                                    Spindexer.disableReversingAuto = false
//                                },
//                                Intake.pulse().withTimeout(1.0).finallyRun {
//                                    Intake.deploy()
//                                }
//                            )
//                        ).withTimeout(3.25).finallyRun { pathPercentage = 0.0 }
//                    )
//                ),
//                parallelCommand(
//                    runOnceCommand {
//                        Intake.deploy()
//                        Intake.intakeState = Intake.IntakeState.INTAKING
//                    },
//                    Drive.driveAlongChoreoPath(path.getSplit(2).get(), resetOdometry = false, poseSupplier = Drive.localizer::pose, exitSupplier = { percent, error -> pathPercentage = percent; Turret.lookForwardOverride = percent > 0.1 && percent < 0.75; percent >= 1.0 && error.translation.norm.meters < 0.5.feet  }),
//                    sequenceCommand(
//                        waitUntilCommand { pathPercentage > 0.85 },
//                        runOnceCommand {
//                            Spindexer.disableReversingAuto = true
//                        },
//                        waitUntilCommand { pathPercentage > 0.95 },
//                        parallelCommand(
//                            Shooter.shoot(true),
//                            sequenceCommand(
//                                runOnceCommand {
//                                    Intake.intakeState = Intake.IntakeState.OFF
//                                    Spindexer.disableReversingAuto = false
//                                },
//                                Intake.pulse()
//                            )
//                        ).withTimeout(5.5)
//                    )
//                ),
//                parallelCommand(
//                    Drive.driveAlongChoreoPath(path.getSplit(3).get(), resetOdometry = false, poseSupplier = Drive.localizer::pose),
//                    runOnceCommand {
//                        Turret.lookForwardOverride = false
//                        Intake.deploy()
//                        pathPercentage = 0.0
//                        Intake.disableSpringProtection = false
//                    }
//                )
//            ),
//            sequenceCommand(
//                waitUntilCommand { Intake.finishedHoming }.finallyRun {
//                    Intake.disableSpringProtection = true
//                    Intake.deploy()
//                    Intake.intakeState = Intake.IntakeState.INTAKING
//                    println("Intake finished homing. Running Intake")
//                }.withName("Intake homing"),
//                parallelCommand(
//                    waitUntilCommand { Intake.deployMotor0Error.absoluteValue < Intake.FLEX_THRESHOLD && Intake.deployMotor1Error.absoluteValue < Intake.FLEX_THRESHOLD }.finallyRun {
//                        Intake.disableSpringProtection = false
//                    },
//                    runCommand {
//                        Shooter.rampUpLoop()
//                    }.withName("Shooter ramp up loop").finallyRun {
//                        Turret.lookForwardOverride = false
//                        Intake.deploy()
//                        pathPercentage = 0.0
//                    }
//                )
//
//            )
//        )
//    }
//
//    private fun doubleSwipeOG(doSideToSideFlip: Boolean): Command {
//        val path = paths["LeftSideDoubleSwipe"]!!.sideToSideFlip(doSideToSideFlip)
//        return parallelCommand(
//            sequenceCommand(
//                parallelCommand(
//                    sequenceCommand(
//                        Drive.driveAlongChoreoPath(path.getSplit(0).get(), resetOdometry = false),
//                        Drive.driveAlongChoreoPath(path.getSplit(1).get(), resetOdometry = false, poseSupplier = Drive.localizer::pose),
//                        runOnceCommand {
//                            Intake.intakeState = Intake.IntakeState.OFF
//                        }
//                    ).withName("first driving double swipe auto"),
//                    sequenceCommand(
//                        waitUntilCommand { Intake.finishedHoming },
//                        runOnceCommand {
//                            Intake.deploy()
//                            Intake.intakeState = Intake.IntakeState.INTAKING
//                            println("Intake finished homing. Running Intake")
//                        }
//                    ).withName("Intake homing"),
//                ).withName("First component Double swipe auto"),
//                parallelCommand(
//                    Shooter.shoot(true),
//                    Intake.pulse()
//                ).withTimeout(3.5),
//                parallelCommand(
//                    runOnceCommand {
//                        Intake.deploy()
//                        Intake.intakeState = Intake.IntakeState.INTAKING
//                                   },
//                    Drive.driveAlongChoreoPath(path.getSplit(2).get(), resetOdometry = false, poseSupplier = Drive.localizer::pose, exitSupplier = { percent, error -> percent >= 1.0 && error.translation.norm.meters < 0.5.feet  }),
//                    ),
//                parallelCommand(
//                    Shooter.shoot(true),
//                    sequenceCommand(
//                        runOnceCommand {
//                            Intake.intakeState = Intake.IntakeState.OFF
//                        },
//                        Intake.pulse()
//                    ),
////                    Drive.driveAlongChoreoPath(path.getSplit(3).get(), resetOdometry = false, poseSupplier = Drive.localizer::pose),
//                )
//            ).withName("Double Swipe auto sequence"),
//            runCommand {
//                Shooter.rampUpLoop()
//            }.withName("Shooter ramp up loop").finallyRun {
//                Intake.deploy()
//            }
//        ).withName("Double swipe auto")
//    }
//
//
//    private fun justShoot(): Command {
//        return parallelCommand(
//            Shooter.shoot().beforeWait(5.0),
//            runOnceCommand {
//                Intake.stow()
//            }
//        )
//    }

    fun warmupDriveAlongPath(): Command {
        val warmupPath = paths["LeftSideDoubleSwipe"]!!//.sideToSideFlip(true) //TODO: UNCOMMENT WHEN 2027 CHOREO
        return Drive.driveAlongChoreoPath(warmupPath.getSplit(0).get(), exitSupplier = { percent, error -> percent >= 1.0 || Robot.isEnabled})//.ignoringDisable(true)
    }
}