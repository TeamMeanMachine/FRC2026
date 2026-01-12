package frc.team2471.frc2026

import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.Timer
import org.littletonrobotics.junction.Logger
import kotlin.math.pow

const val g = 9.80665
class FlyingFuel(val x0: Translation3d, val v0: Translation3d) {
    var pos = x0
    var velocity = v0
    var prevVelocity = v0
    var prevAcceleration = Translation3d(0.0, 0.0, -g)
    var prevT = Timer.getFPGATimestamp()

    fun update(numSteps: Int = 1) {
        // Time from last update
        val deltaT = Timer.getFPGATimestamp() - prevT
        // time between each step
        val dt = deltaT / numSteps

        for (i in 0..<numSteps) {
            val acceleration = Translation3d(0.0, 0.0, -g) + calcDragAccel(prevVelocity)

            velocity += ((prevAcceleration + acceleration) / 2.0).times(dt)

            pos += ((prevVelocity + velocity) / 2.0).times(dt)

            prevAcceleration = acceleration
            prevVelocity = velocity
        }

        prevT += deltaT
    }
}

fun logFuel(name: String, vararg fuel: FlyingFuel) {
    val thing = fuel.map { it.pos }.toTypedArray()
    Logger.recordOutput(name, *thing)
}

fun MutableList<FlyingFuel>.removeFuel() {
    this.removeIf { it.pos.z < 0.0 }
}
private fun calcDragAccel(v: Translation3d): Translation3d {
    val length = v.norm
    Logger.recordOutput("VMag", length)
    val newMagnitude = calcDragAccel(length)
    Logger.recordOutput("DMag", newMagnitude)
    return v.times(-newMagnitude / length)

}
private fun calcDragAccel(s: Double): Double {
    //            rho         v^2        Cd       Frontal Area               mass
    return (0.5 * 1.2 * s.pow(2) * 0.47 * 0.075.pow(2) * Math.PI) / 0.2172
}