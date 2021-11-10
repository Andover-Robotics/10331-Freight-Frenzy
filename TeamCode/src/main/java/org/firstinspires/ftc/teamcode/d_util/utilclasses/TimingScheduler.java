package org.firstinspires.ftc.teamcode.d_util.utilclasses;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.LinkedList;
import java.util.SortedMap;
import java.util.TreeMap;

public class TimingScheduler {
  private OpMode opMode;
  private SortedMap<Double, Runnable> tasks = new TreeMap<>();

  public TimingScheduler(OpMode opMode) {
    this.opMode = opMode;
  }

  public void defer(double seconds, Runnable task) {
    tasks.put(opMode.getRuntime() + seconds, task);
  }

  public void run() {
    LinkedList<Double> toDelete = new LinkedList<>();
    for (SortedMap.Entry<Double, Runnable> entry : tasks.entrySet()) {
      if (opMode.getRuntime() >= entry.getKey()) {
        entry.getValue().run();
        toDelete.add(entry.getKey());
      }
    }
    for (double d : toDelete) {
      tasks.remove(d);
    }
  }

  public void clearAll() {
    tasks.clear();
  }
}

