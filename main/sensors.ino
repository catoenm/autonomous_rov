
// Turn on surface LED if on water surface
bool on_surface(){
  if(analogRead(SURFACE_PROBE) > SURFACE_PROBE_THRESH) {
    digitalWrite(SURFACE_LED, HIGH);
    return true;

  }
  digitalWrite(SURFACE_LED, LOW);
  return false;
}

// Turn on danger LED if more than 20 degrees pitched
bool danger_zone() {
  if(abs(imu_get_pitch()) > 20) {
    digitalWrite(DANGER_LED, HIGH);
    return true;
  }
  digitalWrite(DANGER_LED, LOW);
  return false;
}

// Reset and start the pressure sensor
void depth_sensor_setup() {
  depth_sensor.reset();
  depth_sensor.begin();

  // TODO: Write a calibration routine for depth sensor
  //       Take 5s of data above water to get relative
  pressure_baseline = depth_sensor.getPressure(ADC_4096);
}

// Update relative and absolute pressure values
double update_pressure() {
  pressure_abs = depth_sensor.getPressure(ADC_4096);
  pressure_relative = sealevel(pressure_abs, BASE_ALTITUDE);
}

// Given a pressure P (mbar) taken at a specific altitude (meters),
// return the equivalent pressure (mbar) at sea level.
// This produces pressure readings that can be used for weather measurements.
double sealevel(double P, double A) {
 return(P/pow(1-(A/44330.0),5.255));
}
