package org.firstinspires.ftc.teamcode.extraneous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.LinkedList;
import java.util.Queue;

public class ActionSchedular {
    final Queue<Action> actions = new LinkedList<>();

    final FtcDashboard dash = FtcDashboard.getInstance();
    final Canvas canvas = new Canvas();

    public void addAction(Action action) {
        actions.add(action);
    }

    // Won't generate previews
    public void run() {
        if (actions.peek() != null) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            boolean running = actions.peek().run(packet);
            dash.sendTelemetryPacket(packet);

            if (!running) {
                actions.remove();
            }
        }
    }

    public void runEndless() {
        if (actions.peek() != null) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().getOperations().addAll(canvas.getOperations());

            dash.sendTelemetryPacket(packet);
        }
    }


}
