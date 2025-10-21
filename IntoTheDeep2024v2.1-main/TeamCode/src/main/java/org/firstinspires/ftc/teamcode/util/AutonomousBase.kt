package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.util.ElapsedTime
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.pedro.PedroOpMode
import kotlin.math.roundToInt

open class AutonomousBase(vararg sbst: Subsystem = arrayOf()) : PedroOpMode(*sbst) {
    private var runtime: ElapsedTime = ElapsedTime()
    private var loopTime: ElapsedTime = ElapsedTime()

    override fun onInit() {
        this.telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        telemetry.msTransmissionInterval = 10
        OpModeData.telemetry = this.telemetry

        runtime.reset()
        loopTime.reset()

        this.initialize()
    }

    override fun onUpdate() {
        this.update()

        telemetry.addData("Runtime", "${runtime.seconds().roundToInt()} seconds")
        telemetry.addData("Loop Time", "${loopTime.milliseconds().roundToInt()} ms")
        telemetry.addLine()
        
        loopTime.reset()

        telemetry.update()
    }

    open fun initialize() {}

    open fun update() {}
}