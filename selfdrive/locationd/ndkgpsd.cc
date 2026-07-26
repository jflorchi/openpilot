#include <pthread.h>
#include <signal.h>
#include <hardware/gps.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>
#include <atomic>
#include <mutex>
#include "cereal/messaging/messaging.h"

// Constants for coordinate to meter conversion
static constexpr double METERS_PER_DEG_LAT = 110574.0;
static constexpr double METERS_PER_DEG_LON_EQUATOR = 111320.0;
static constexpr double DEG_TO_RAD = M_PI / 180.0;
static constexpr int GPS_UPDATE_INTERVAL_MS = 1000;

// Thread-safe state management
static std::mutex gStateMutex;
static PubMaster pm({"gpsLocationExternal"});
static const GpsInterface* gGpsInterface = nullptr;
static std::atomic<bool> gRunning{true};

// Previous location state (protected by gStateMutex)
struct LocationState {
  double latitude = 0.0;
  double longitude = 0.0;
  double altitude = 0.0;
  int64_t timestamp = 0;
  bool initialized = false;
};
static LocationState gLastLocation;

static void locationCallback(GpsLocation* location) {
  if (!location) {
    fprintf(stderr, "Received null location\n");
    return;
  }

  MessageBuilder msg_builder;
  auto gpsLoc = msg_builder.initEvent().initGpsLocationExternal();

  gpsLoc.setSource(cereal::GpsLocationData::SensorSource::UBLOX);
  gpsLoc.setFlags(location->flags);
  gpsLoc.setLatitude(location->latitude);
  gpsLoc.setLongitude(location->longitude);
  gpsLoc.setAltitude(location->altitude);
  gpsLoc.setSpeed(location->speed);
  gpsLoc.setBearingDeg(location->bearing);
  gpsLoc.setAccuracy(location->accuracy);
  gpsLoc.setTimestamp(static_cast<int64_t>(location->timestamp));

  // Calculate velocity components with thread safety
  float vel_n = 0.0f;
  float vel_e = 0.0f;
  float vel_d = 0.0f;

  {
    std::lock_guard<std::mutex> lock(gStateMutex);

    if (gLastLocation.initialized) {
      double elapsed_sec = (location->timestamp - gLastLocation.timestamp) / 1000.0;

      // Only calculate velocity if we have a meaningful time delta
      if (elapsed_sec > 0.1) {
        double dlat = location->latitude - gLastLocation.latitude;
        double dlon = location->longitude - gLastLocation.longitude;
        double dalt = location->altitude - gLastLocation.altitude;

        // Use average latitude for longitude conversion
        double avg_lat_rad = ((location->latitude + gLastLocation.latitude) / 2.0) * DEG_TO_RAD;

        // Convert to meters and calculate velocity
        vel_n = static_cast<float>((dlat * METERS_PER_DEG_LAT) / elapsed_sec);
        vel_e = static_cast<float>((dlon * METERS_PER_DEG_LON_EQUATOR * cos(avg_lat_rad)) / elapsed_sec);
        vel_d = static_cast<float>(-dalt / elapsed_sec);  // Down is positive in NED
      }
    }

    // Update last location state
    gLastLocation.latitude = location->latitude;
    gLastLocation.longitude = location->longitude;
    gLastLocation.altitude = location->altitude;
    gLastLocation.timestamp = static_cast<int64_t>(location->timestamp);
    gLastLocation.initialized = true;
  }

  float vned[] = {vel_n, vel_e, vel_d};
  gpsLoc.setVNED(vned);

  gpsLoc.setVerticalAccuracy(location->accuracy);
  gpsLoc.setSpeedAccuracy(location->accuracy);
  gpsLoc.setBearingAccuracyDeg(location->accuracy);

  pm.send("gpsLocationExternal", msg_builder);
}

static void statusCallback(GpsStatus* status) {
  if (!status) return;

  const char* status_str = "unknown";
  switch (status->status) {
    case GPS_STATUS_NONE:
      status_str = "none";
      break;
    case GPS_STATUS_SESSION_BEGIN:
      status_str = "session begin";
      break;
    case GPS_STATUS_SESSION_END:
      status_str = "session end";
      break;
    case GPS_STATUS_ENGINE_ON:
      status_str = "engine on";
      break;
    case GPS_STATUS_ENGINE_OFF:
      status_str = "engine off";
      break;
  }
  fprintf(stdout, "GPS status: %s\n", status_str);
}

static void svStatusCallback(GpsSvStatus* sv_info) {
  // Satellite vehicle status - could log satellite count for debugging
  if (sv_info) {
    fprintf(stdout, "Satellites in view: %d\n", sv_info->num_svs);
  }
}

static void nmeaCallback(GpsUtcTime timestamp, const char* nmea, int length) {
  // NMEA sentences available for debugging if needed
  (void)timestamp;
  (void)nmea;
  (void)length;
}

static void setCapabilitiesCallback(uint32_t capabilities) {
  fprintf(stdout, "GPS capabilities: 0x%08x\n", capabilities);
}

static void acquireWakelockCallback() {
  // Acquire wakelock to keep device awake during GPS operation
  // Implementation depends on platform
}

static void releaseWakelockCallback() {
  // Release wakelock
  // Implementation depends on platform
}

static pthread_t createThreadCallback(const char* name, void (*start)(void*), void* arg) {
  pthread_t thread;
  pthread_attr_t attr;

  int err = pthread_attr_init(&attr);
  if (err != 0) {
    fprintf(stderr, "Failed to init thread attr for %s: %d\n", name, err);
    return 0;
  }

  err = pthread_create(&thread, &attr, reinterpret_cast<void*(*)(void*)>(start), arg);
  if (err != 0) {
    fprintf(stderr, "Failed to create thread %s: %d\n", name, err);
    pthread_attr_destroy(&attr);
    return 0;
  }

  pthread_attr_destroy(&attr);
  fprintf(stdout, "Created GPS thread: %s\n", name);
  return thread;
}

static void sigintHandler(int signum) {
  (void)signum;
  fprintf(stdout, "\nShutting down GPS...\n");
  gRunning = false;

  if (gGpsInterface) {
    gGpsInterface->stop();
    gGpsInterface->cleanup();
  }
}

static GpsCallbacks callbacks = {
  sizeof(GpsCallbacks),
  locationCallback,
  statusCallback,
  svStatusCallback,
  nmeaCallback,
  setCapabilitiesCallback,
  acquireWakelockCallback,
  releaseWakelockCallback,
  createThreadCallback,
};

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  // Set up signal handlers
  struct sigaction sa;
  sa.sa_handler = sigintHandler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  // Load GPS hardware module
  hw_module_t* module = nullptr;
  int err = hw_get_module(GPS_HARDWARE_MODULE_ID, const_cast<hw_module_t const**>(&module));
  if (err != 0) {
    fprintf(stderr, "Failed to load GPS module: %d\n", err);
    return 1;
  }

  // Open GPS device
  hw_device_t* device = nullptr;
  err = module->methods->open(module, GPS_HARDWARE_MODULE_ID, &device);
  if (err != 0) {
    fprintf(stderr, "Failed to open GPS device: %d\n", err);
    return 1;
  }

  gps_device_t* gps_device = reinterpret_cast<gps_device_t*>(device);
  gGpsInterface = gps_device->get_gps_interface(gps_device);

  if (!gGpsInterface) {
    fprintf(stderr, "Failed to get GPS interface\n");
    return 1;
  }

  // Initialize GPS
  err = gGpsInterface->init(&callbacks);
  if (err != 0) {
    fprintf(stderr, "Failed to initialize GPS: %d\n", err);
    return 1;
  }

  fprintf(stdout, "GPS initialized successfully\n");

  // Clear any stale aiding data and start GPS
  gGpsInterface->delete_aiding_data(GPS_DELETE_ALL);
  err = gGpsInterface->start();
  if (err != 0) {
    fprintf(stderr, "Failed to start GPS: %d\n", err);
    gGpsInterface->cleanup();
    return 1;
  }

  // Configure position mode: MS-Based with periodic updates
  err = gGpsInterface->set_position_mode(
      GPS_POSITION_MODE_MS_BASED,
      GPS_POSITION_RECURRENCE_PERIODIC,
      GPS_UPDATE_INTERVAL_MS,
      0,  // preferred accuracy (0 = no preference)
      0   // preferred time (0 = no preference)
  );
  if (err != 0) {
    fprintf(stderr, "Warning: Failed to set position mode: %d\n", err);
    // Continue anyway, GPS may still work with default mode
  }

  fprintf(stdout, "GPS started, publishing to 'gpsLocationExternal'\n");

  // Main loop - wait for shutdown signal
  while (gRunning) {
    sleep(1);
  }

  fprintf(stdout, "GPS publisher shutdown complete\n");
  return 0;
}
