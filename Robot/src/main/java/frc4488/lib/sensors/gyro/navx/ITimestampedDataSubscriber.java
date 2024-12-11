package frc4488.lib.sensors.gyro.navx;

import frc4488.lib.sensors.gyro.navx.AHRSProtocol.AHRSUpdateBase;

/**
 * The ITimestampedDataSubscriber interface provides a method for consumers of navX-Model device
 * data to be rapidly notified whenever new data has arrived.
 *
 * <p>- timestampedDataReceived(): reception of sensor-timestamped data
 *
 * <p>A "sensor" timestamp is provided, generated by the navX-Model device, which is at millisecond
 * resolution. A "system timestamp", also at millisecond resolution, is also provided, which
 * represents as accurately as possible the time at which the data was acquired from the navX-Model
 * device. Note that the "system timestamp" typically has more jitter since it is generated by the
 * host of the navX-Model device.
 *
 * <p>Thus, in general sensor timestamps are preferred, as they are generated by the navX-Model
 * device motion processor and has a greater accuracy (+/- 1ms) than the system timestamp which is
 * vulnerable to latencies introduced by the host operating system.
 *
 * <p>The system timestamp is provided to allow performance monitoring of the navX-Model device
 * host's data acquisition process.
 */
public interface ITimestampedDataSubscriber {
  public void timestampedDataReceived(
      long system_timestamp, long sensor_timestamp, AHRSUpdateBase sensor_data, Object context);
}
