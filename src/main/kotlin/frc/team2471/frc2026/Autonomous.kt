package frc.team2471.frc2026

//import frc.team2471.frc2026.tests.*
import org.team2471.frc.lib.autonomous.Autonomi
import org.team2471.frc.lib.autonomous.AutoRoutine
import org.team2471.frc.lib.autonomous.TestRoutine
import org.team2471.frc.lib.commands.onCancel
import org.team2471.frc.lib.commands.parallel
import org.team2471.frc.lib.commands.periodic
import org.team2471.frc.lib.commands.periodicTimeout
import org.team2471.frc.lib.commands.use
import org.team2471.frc.lib.commands.useUnnamed
import org.team2471.frc.lib.math.round
import org.team2471.frc.lib.units.feet
import org.team2471.frc.lib.units.meters
import org.team2471.frc.lib.units.seconds
import org.wpilib.command3.Command
import org.wpilib.command3.Scheduler
import org.wpilib.hardware.hal.RobotMode
import org.wpilib.math.geometry.Pose2d
import kotlin.math.absoluteValue


object Autonomous: Autonomi() {

//    val paths: MutableMap<String, Trajectory<SwerveSample>> = findChoreoPaths()  <-- already inside AutoMaker

    override val autos: List<AutoRoutine> = listOf(
        AutoRoutine("Eight Foot Straight", eightFootStraight()),
        AutoRoutine("Square Path Test", squarePathTest()),
        AutoRoutine("Double Swipe Left", doubleSwipe(false)),
        AutoRoutine("Double Swipe Right", doubleSwipe(true)),
        AutoRoutine("Print for 20 seconds", printFor20Seconds()),
    )

    override val tests: List<TestRoutine> = listOf(
        TestRoutine("Drive Set Angle Offsets", Drive.setAngleOffsets(), { Robot.disableAllDefaultCommands() }),
    )

    /** Supplier that sets the robot's pose. */
    override val drivePoseSetter: (Pose2d) -> Unit = { Drive.pose = it }

    /** Warmup function for speeding up auto loop times. Runs when selected auto changes. */
    override val warmupFunction: () -> Unit = {
        println("scheduling auto warmup")
        Scheduler.getDefault().schedule(warmupDriveAlongPath())
        println("finished scheduling auto warmup")
    }


    init {
        println("Autonomous init")

        // Register Auto OpModes (This adds them to the DS chooser)
        autos.forEach {
            Robot.addOpModeFactory({ it.toAutoOpMode() }, RobotMode.AUTONOMOUS, it.name)
            println("Registered ${it.name} as an AutoOpMode")
        }
        tests.forEach {
            Robot.addOpModeFactory({ it.toTestOpMode() }, RobotMode.UTILITY, it.name)
            println("Registered ${it.name} TestOpMode")
        }
        Robot.publishOpModes()

        println("Autonomous path count: ${paths.size}")
    }

    /** Autonomous commands */

    private fun printFor20Seconds() = use(Drive) {
        println("starting printFor20Seconds")
        periodicTimeout(20.0) {
            println("dt time: ${it.round(2)}")
        }
        println("finished")
    }

    private fun eightFootStraight() = use(Drive) {
        await(Drive.driveAlongChoreoPath(paths["eightFoot"]!!, resetOdometry = true))
    }

    private fun squarePathTest() = use(Drive) {
        await(Drive.driveAlongChoreoPath(paths["square"]!!, resetOdometry = true))
    }

    private fun doubleSwipe(doSideToSideFlip: Boolean) = use {
        val path = paths["LeftSideDoubleSwipe"]!!//.sideToSideFlip(doSideToSideFlip)
        var pathPercentage = 0.0
        parallel({

            Turret.lookForwardOverride = true
            // Beginning Intake Path
            await(Drive.driveAlongChoreoPath(path.getSplit(0).get(), resetOdometry = false))

            // Rest of intake path going back
            parallel({
                await(Drive.driveAlongChoreoPath(path.getSplit(1).get(), resetOdometry = false, poseSupplier = Drive.localizer::pose, exitSupplier = { percent, error -> pathPercentage = percent; Turret.lookForwardOverride = percent < 0.75; percent >= 1.0 }))
            }, {
                waitUntil { pathPercentage > 0.8 }
                // Disable spindexer reversing a little early
                Spindexer.disableReversingAuto = true
                waitUntil { pathPercentage > 0.9 }
                // start shooting slightly early
                parallel({
                    await(Shooter.shoot().withTimeout(3.25.seconds))
                }, {
                    Intake.intakeState = Intake.IntakeState.OFF
                    Spindexer.disableReversingAuto = false
                    await(Intake.pulse().withTimeout(1.0.seconds))
                    Intake.deploy()
                })
                pathPercentage = 0.0
            })

            // Next Intake Path out and back
            parallel({
                await(Drive.driveAlongChoreoPath(path.getSplit(2).get(), resetOdometry = false, poseSupplier = Drive.localizer::pose, exitSupplier = { percent, error -> pathPercentage = percent; Turret.lookForwardOverride = percent > 0.1 && percent < 0.75; percent >= 1.0 && error.translation.norm.meters < 0.5.feet  }))
            }, {
                Intake.deploy()
                Intake.intakeState = Intake.IntakeState.INTAKING

                waitUntil { pathPercentage > 0.85 }
                // Disable spindexer reversing a little early
                Spindexer.disableReversingAuto = true
                waitUntil { pathPercentage > 0.95 }
                // Start shooting early
                parallel({
                    await(Shooter.shoot(true).withTimeout(5.5.seconds))
                }, {
                    Intake.intakeState = Intake.IntakeState.OFF
                    Spindexer.disableReversingAuto = false
                    await(Intake.pulse().withTimeout(5.5.seconds))
                })
            })
            // End of shooting, sprint out to middle
            Turret.lookForwardOverride = false
            Intake.deploy()
            pathPercentage = 0.0
            Intake.disableSpringProtection = false
            await(Drive.driveAlongChoreoPath(path.getSplit(3).get(), resetOdometry = false, poseSupplier = Drive.localizer::pose))
        }, {
            // Wait until intake is done homing
            waitUntil { Intake.finishedHoming }
            // Deploy and run intake
            Intake.deploy()
            Intake.intakeState = Intake.IntakeState.INTAKING
            println("Intake finished homing. Running Intake")

            parallel({
                // Wait until intake is out to enable spring protetion
                waitUntil { Intake.deployMotor0Error.absoluteValue < Intake.FLEX_THRESHOLD && Intake.deployMotor1Error.absoluteValue < Intake.FLEX_THRESHOLD }
                Intake.disableSpringProtection = false
            }, {
                // Ramp up shooter always
                this.periodic {
                    Shooter.rampUpLoop()
                }
            })
        })
    }.onCancel {
        Turret.lookForwardOverride = false
        Intake.deploy()
    }

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

    fun warmupDriveAlongPath() = useUnnamed(Drive) {
//        val warmupPath = paths["eightFoot"]!!//.sideToSideFlip(true) //TODO: UNCOMMENT WHEN 2027 CHOREO
//        await(Drive.driveAlongChoreoPath(warmupPath.getSplit(0).get(), exitSupplier = { percent, error -> percent >= 1.0 || Robot.isEnabled}))
        println("Warmup DAL")
    }.withPriority(Command.LOWEST_PRIORITY + 1).named("Warmup Drive Along Path")
}