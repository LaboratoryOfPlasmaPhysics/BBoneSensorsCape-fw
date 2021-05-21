#include <DallasTemperature.h>
#include <OneWire.h>
#include <Adafruit_SHTC3.h>

#include <array>
#include <tuple>


template<int... pins>
struct DS18B20_driver_t {
  static inline constexpr auto wire_count = sizeof...(pins);
  
  std::array<OneWire, wire_count> wire;
  std::array<DallasTemperature, wire_count> sensors;

  DS18B20_driver_t()
    : wire{pins...} {
  }
  void begin()
  {
    for (unsigned char i = 0; i < wire_count; i++)
    {
      sensors[i].setOneWire(&(wire[i]));
      sensors[i].begin();
      sensors[i].setResolution(12);
    }

  }

  inline void requestTemperatures()
  {
    for (unsigned char i = 0; i < wire_count; i++)
    {
      sensors[i].requestTemperatures();
    }
  }
  
  inline float getTempC(unsigned int wire_index, unsigned char* SN) {
    return sensors[wire_index].getTempC(SN);
  }

  inline unsigned char getDeviceCount(unsigned int wire_index)
  {
    return sensors[wire_index].getDeviceCount();
  }

  inline void getAddress(unsigned int wire_index, unsigned char* SN, unsigned char dev_index)
  {
    sensors[wire_index].getAddress(SN,dev_index);
  }
  
};

DS18B20_driver_t<16, 17, 5, 15> drivers{};
Adafruit_SHTC3 shtc3 = Adafruit_SHTC3();

struct DS18B20_t {
  float temperature = 0.;
  unsigned char wire_index = 255;
  unsigned char SN[8];
};

inline void getTempC(DS18B20_t& measurement) {
  measurement.temperature = drivers.getTempC(measurement.wire_index,measurement.SN);
}


std::array<DS18B20_t, 128> measurements;
sensors_event_t humidity, internal_temperature;

unsigned char sensor_count = 0;

template<typename T>
struct CSV_printer {
  CSV_printer(T& measurements) {}

  void print_SN(unsigned char* SN)
  {
    for (int i = 0; i < 8; i++)
    {
      Serial.print(SN[i], HEX);
    }
  }

  void print_header() {
    Serial.print("Internal_temp\tHumidity\t");
    for (auto i = 0U; i < sensor_count; i++) {
      print_SN(measurements[i].SN);
      Serial.print('\t');
    }
    Serial.println();
  }
  void print_values()
  {
    Serial.print(internal_temperature.temperature);
    Serial.print('\t');
    Serial.print(humidity.relative_humidity);
    for (auto i = 0U; i < sensor_count; i++) {    
      Serial.print('\t');
      Serial.print(measurements[i].temperature);
    }
    Serial.println();
  }
};

template<typename T>
struct Graphite_printer {
  Graphite_printer(T& measurements) {}

  static void print_SN(const unsigned char* SN)
  {
    for (int i = 0; i < 8; i++)
    {
      Serial.print(SN[i], HEX);
    }
  }

  void print_header() {
  }

  template<typename U>
  static void print_metric(const char* name,const U& value)
  {
    Serial.print(name);
    Serial.print('\t');
    Serial.println(value);
  }

  static void print_metric(const DS18B20_t& measurement)
  {
    Serial.print("DS18B20.");
    print_SN(measurement.SN);
    Serial.print('\t');
    Serial.println(measurement.temperature);
  }
  
  static void print_values()
  {
    print_metric("SHTC3.Internal_temp",internal_temperature.temperature);
    print_metric("SHTC3.Humidity",humidity.relative_humidity);
    for (auto i = 0U; i < sensor_count; i++) { 
      print_metric(measurements[i]);
    }
  }
};

//CSV_printer printer{measurements};
Graphite_printer<decltype(measurements)> printer{measurements};

void setup(void) {
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(8000);
  drivers.begin();
  Wire.begin();
  shtc3.begin();
  for (unsigned char wire_index = 0; wire_index < decltype(drivers)::wire_count; wire_index++) {
    Serial.print("# Scanning wire:");
    Serial.println((int)wire_index);
    const auto dev_count = drivers.getDeviceCount(wire_index);
    unsigned char dev_index = 0;
    Serial.print("# Found: ");
    Serial.print((int)dev_count);
    Serial.println(" sensors");
    while (dev_index < dev_count) {
      auto& measurement = measurements[sensor_count];
      measurement.wire_index = wire_index;
      drivers.getAddress(wire_index, measurement.SN, dev_index);
      sensor_count++;
      dev_index++;
    }
  }
  printer.print_header();
}

void loop(void) {
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  delay(2000);
  drivers.requestTemperatures();
  delay(2000);
  for (int i = 0; i < sensor_count; i++)
  {
    auto& measurement = measurements[i];
    getTempC(measurement);
  }
  shtc3.getEvent(&humidity, &internal_temperature);// populate temp and humidity objects with fresh data
  printer.print_values();
}
