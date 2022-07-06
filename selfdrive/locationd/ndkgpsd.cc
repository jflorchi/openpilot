#include <pthread.h>
#include <signal.h>
#include <hardware/gps.h>
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <math.h>

#include "cereal/messaging/messaging.h"
#include "selfdrive/common/swaglog.h"
#include "selfdrive/common/util.h"

static PubMaster pm({"gpsLocationExternal"});
static const GpsInterface* gGpsInterface = NULL;

static float lastLat = 0;
static float lastLong = 0;
static float lastAlt = 0;
static long lastTime = 0;

static void locationCallback(GpsLocation* location) {
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
  gpsLoc.setTimestamp((long) location->timestamp);

  float dlong = (location->longitude - lastLong);
  float dlat = (location->latitude - lastLat);
  float rdlat = dlat * (M_PI / 180.0);

  long elapsed = (location->timestamp - lastTime) / 1000.0;
  float vel_n = (dlat * 110574) / elapsed;
  float vel_e = (dlong * 111320 * cos(rdlat)) / elapsed;
  float f[] = { vel_n, vel_e, location->speed };
  gpsLoc.setVNED(f);
  gpsLoc.setVerticalAccuracy(location->accuracy);
  gpsLoc.setSpeedAccuracy(location->accuracy);
  gpsLoc.setBearingAccuracyDeg(location->accuracy);

  lastLat = location->latitude;
  lastLong = location->longitude;
  lastAlt = location->altitude;
  lastTime = (long) location->timestamp;

  pm.send("gpsLocationExternal", msg_builder);
}

static void statusCallback(GpsStatus* status) { }

static void svStatusCallback(GpsSvStatus* sv_info) { }

static void nmeaCallback(GpsUtcTime timestamp, const char* nmea, int length) {
//  fprintf(stdout, "*** nmea info\n");
//  fprintf(stdout, "timestamp:\t%ld\n", (long)timestamp);
//  fprintf(stdout, "nmea:     \t%s\n", nmea);
//  fprintf(stdout, "length:   \t%d\n", length);
}

static void setCapabilitiesCallback(uint32_t capabilities) { }

static void getWakelockCallback() { }

static void releaseWakelockCallback() { }

static pthread_t createThreadCallback(const char* name, void (*start)(void *), void* arg) {
  pthread_t thread;
  pthread_attr_t attr;
  int err;

  err = pthread_attr_init(&attr);
  err = pthread_create(&thread, &attr, (void*(*)(void*))start, arg);

  return thread;
}

static void sigintHandler(int signum) {
  if (gGpsInterface) {
    gGpsInterface->stop();
    gGpsInterface->cleanup();
  }
}

GpsCallbacks callbacks = {
  sizeof(GpsCallbacks),
  locationCallback,
  statusCallback,
  svStatusCallback,
  nmeaCallback,
  setCapabilitiesCallback,
  getWakelockCallback,
  releaseWakelockCallback,
  createThreadCallback,
};

int main(int argc, char *argv[]) {
  LOGW("starting ndk_gps");

  LOGW("setting up signal handler");
  signal(SIGINT, sigintHandler);

  LOGW("create gps interface");

  int err;
  hw_module_t* module;
  err = hw_get_module(GPS_HARDWARE_MODULE_ID, (hw_module_t const**)&module);
  if (!err) {
    hw_device_t* device;
    err = module->methods->open(module, GPS_HARDWARE_MODULE_ID, &device);
    if (!err) {
      gps_device_t* gps_device = (gps_device_t *)device;
      gGpsInterface = gps_device->get_gps_interface(gps_device);
    }
  }

  LOGW("initailized gps interface");
  if (gGpsInterface && !gGpsInterface->init(&callbacks)) {
    LOGW("starting gps tracker");
    gGpsInterface->delete_aiding_data(GPS_DELETE_ALL);
    gGpsInterface->start();
    gGpsInterface->set_position_mode(GPS_POSITION_MODE_MS_BASED, GPS_POSITION_RECURRENCE_PERIODIC, 1000, 0, 0);
  }
  while (true) {
    sleep(500);
  }
  return 0;
}
