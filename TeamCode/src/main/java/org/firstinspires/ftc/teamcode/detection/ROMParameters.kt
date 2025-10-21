package org.firstinspires.ftc.teamcode.detection

import com.acmerobotics.dashboard.config.Config

@Config
object ROMParameters {
    var turretMaxAngleDeviation: Double = Math.toRadians(40.0)
    var turretMaxPositionDeviation: Double = 0.203
    var clawYawMaxAngleDeviation: Double = Math.toRadians(90.0)
    var clawYawMaxPositionDeviation: Double = 0.29
    var armLength: Double = 19.0
    var ticksPerCm: Double = 38.2
}
