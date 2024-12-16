package frc4488.lib.devices;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.Filesystem;
import frc4488.lib.math.EpsilonUtil;
import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/** Uses the CTRE {@link Orchestra} internally to play music on {@link TalonFX}s */
public class Conductor {

  private static final File SONG_FOLDER = new File(Filesystem.getDeployDirectory(), "songs");

  private final Orchestra orchestra;
  private final List<String> songs;
  private final Set<ConductorEventListener> listeners;
  private int nextChannel;
  private String lastLoadedSong;

  public Conductor() {
    orchestra = new Orchestra();
    songs = new ArrayList<>();
    String[] songFiles = SONG_FOLDER.list();
    if (songFiles != null) {
      for (String song : songFiles) {
        if (song.endsWith(".chrp")) {
          songs.add(song.substring(0, song.length() - ".chrp".length()));
        }
      }
    }
    songs.sort(Comparator.naturalOrder());
    listeners = new HashSet<>();
    nextChannel = 0;
    lastLoadedSong = null;
  }

  public List<String> getSongs() {
    return Collections.unmodifiableList(songs);
  }

  public boolean addEventListener(ConductorEventListener listener) {
    return listeners.add(listener);
  }

  public boolean removeEventListener(ConductorEventListener listener) {
    return listeners.remove(listener);
  }

  /**
   * Consider also enabling AllowMusicDurDisable in the AudioConfig
   *
   * @see #addInstrument(TalonFX, boolean)
   */
  public void addInstrument(ParentDevice motor) {
    orchestra.addInstrument(motor, nextChannel);
    nextChannel++;
  }

  /**
   * Note that this will override the previous audio config
   *
   * @see #addInstrument(ParentDevice)
   */
  public void addInstrument(TalonFX motor, boolean allowMusicDurDisable) {
    AudioConfigs config = new AudioConfigs();
    config.AllowMusicDurDisable = allowMusicDurDisable;
    motor.getConfigurator().apply(config);
    addInstrument(motor);
  }

  public void clearInstruments() {
    orchestra.clearInstruments();
    nextChannel = 0;
  }

  public boolean play(String song) {
    if (!songs.contains(song)) {
      return false;
    }

    if (song.equals(lastLoadedSong)) {
      orchestra.stop(); // Reset to the beginning of the song
    } else {
      orchestra.loadMusic(new File(SONG_FOLDER, song + ".chrp").getAbsolutePath());
      lastLoadedSong = song;
    }

    listeners.forEach(listener -> listener.onPlay(song));
    orchestra.play();
    return true;
  }

  public boolean pause() {
    if (!orchestra.isPlaying()) {
      return false;
    }
    listeners.forEach(ConductorEventListener::onPause);
    orchestra.pause();
    return true;
  }

  public boolean resume() {
    if (orchestra.isPlaying()
        || lastLoadedSong == null
        || EpsilonUtil.epsilonEquals(orchestra.getCurrentTime(), 0)) {
      return false;
    }
    listeners.forEach(ConductorEventListener::onResume);
    orchestra.play();
    return true;
  }

  public boolean stop() {
    if (!orchestra.isPlaying()) {
      return false;
    }
    listeners.forEach(ConductorEventListener::onStop);
    orchestra.stop();
    return true;
  }

  public boolean isPlaying() {
    return orchestra.isPlaying();
  }
}
