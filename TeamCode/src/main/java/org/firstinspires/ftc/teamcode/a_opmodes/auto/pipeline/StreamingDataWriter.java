package org.firstinspires.ftc.teamcode.a_opmodes.auto.pipeline;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.util.Locale;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import java.util.stream.DoubleStream;

public class StreamingDataWriter extends TimerTask {
  private final Timer timer = new Timer();
  private final Supplier<double[]> source;
  private final int freq;
  private final PrintWriter out;
  private final AtomicBoolean running = new AtomicBoolean(true);

  public StreamingDataWriter(int freq, Supplier<double[]> source, String filename)
      throws FileNotFoundException {
    this.source = source;
    this.freq = freq;
    out = new PrintWriter(filename);
  }

  public void startStreaming() {
    timer.scheduleAtFixedRate(this, 0, 1000 / freq);
  }

  public void pause() {
    running.set(false);
  }

  public void resume() {
    running.set(true);
  }

  public boolean paused() {
    return !running.get();
  }

  public void stopStreaming() {
    timer.cancel();
    out.close();
  }

  public void run() {
    if (running.get()) {
      String line = DoubleStream.of(source.get())
          .mapToObj(it -> String.format(Locale.US, "%.5f", it))
          .collect(Collectors.joining(","));
      out.println(line);
    }
  }

  public void writeLine(String line) {
    out.println(line);
  }
}
