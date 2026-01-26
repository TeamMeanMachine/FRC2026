package frc.team2471.frc2026

import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.Timer
import frc.team2471.frc2026.AimUtils.AIR_DENSITY
import frc.team2471.frc2026.AimUtils.FUEL_DRAG_COEFFICIENT
import frc.team2471.frc2026.AimUtils.FUEL_FRONTAL_AREA
import frc.team2471.frc2026.AimUtils.FUEL_MASS
import frc.team2471.frc2026.AimUtils.G
import org.littletonrobotics.junction.Logger
import kotlin.math.pow

class FuelSim(val x0: Translation3d, val v0: Translation3d) {
    var pos = x0
    var velocity = v0
    var prevVelocity = v0
    var prevAcceleration = Translation3d(0.0, 0.0, -G)
    var prevT = Timer.getFPGATimestamp()

    fun update(numSteps: Int = 1) {
        // Time from last update
        val deltaT = Timer.getFPGATimestamp() - prevT


        update(deltaT, numSteps)
    }

    fun update(deltaT: Double, numSteps: Int) {
        // time between each step
        val dt = deltaT / numSteps

        for (i in 0..<numSteps) {
            val acceleration = Translation3d(0.0, 0.0, -G) + calcDragAccel(prevVelocity)

            velocity += ((prevAcceleration + acceleration) / 2.0).times(dt)

            pos += ((prevVelocity + velocity) / 2.0).times(dt)

            prevAcceleration = acceleration
            prevVelocity = velocity
        }

        prevT += deltaT
    }
}

fun logFuel(name: String, vararg fuel: FuelSim) {
    val thing = fuel.map { it.pos }.toTypedArray()
    Logger.recordOutput(name, *thing)
}

fun MutableList<FuelSim>.removeFuel() {
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
    //               rho           v^2                          Cd       Frontal Area          mass
    return (0.5 * AIR_DENSITY * s.pow(2) * FUEL_DRAG_COEFFICIENT * FUEL_FRONTAL_AREA) / FUEL_MASS
}
