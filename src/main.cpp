// MIT License
// https://github.com/gonzalocasas/arduino-uno-dragino-lorawan/blob/master/LICENSE
// Based on examples from https://github.com/matthijskooijman/arduino-lmic
// Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman

#include <Arduino.h>
#include "lmic.h"
#include <hal/hal.h>
#include <SPI.h>
#include <SSD1306.h>
#include "soc/efuse_reg.h"
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
#include "Adafruit_PM25AQI.h"

#define LEDPIN 2

#define OLED_I2C_ADDR 0x3C
#define OLED_RESET 16
#define OLED_SDA 4
#define OLED_SCL 15

#define uS_TO_S_FACTOR 1000000  //Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  20        //Time ESP32 will go to sleep (in seconds)

RTC_DATA_ATTR int16_t counter = 0;
RTC_DATA_ATTR float h = 0;
RTC_DATA_ATTR float t = 0;
RTC_DATA_ATTR float hic = 0;

#define SDA 21
#define SCL 13
TwoWire I2Cone = TwoWire(1);

SSD1306 display (OLED_I2C_ADDR, OLED_SDA, OLED_SCL);

Adafruit_PM25AQI aqi = Adafruit_PM25AQI();

// BME680
#define SEALEVELPRESSURE_HPA (1020)
//#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME680 bme; // I2C
float hum_weighting = 0.25; // so hum effect is 25% of the total air quality score
float gas_weighting = 0.75; // so gas effect is 75% of the total air quality score

float hum_score, gas_score;
float gas_reference = 250000;
float hum_reference = 40;
int   getgasreference_count = 0;

// Change here your lorawan keys
static u1_t NWKSKEY[16] = { 0xfe, 0xe3, 0xd0, 0xfc, 0x4b, 0xf2, 0xb5, 0x8d, 0x8f, 0x76, 0x99, 0x72, 0xf6, 0xe6, 0x7d, 0x2e };  // Paste here the key in MSB format
static u1_t APPSKEY[16] = { 0xce, 0x9e, 0x5f, 0xeb, 0x3b, 0x72, 0x4a, 0xac, 0x8f, 0xea, 0xa9, 0xba, 0x47, 0xba, 0x99, 0x36 };  // Paste here the key in MSB format
//static u4_t DEVADDR = 0x0323a162;   // ttgo-bme680-1
static u4_t DEVADDR = 0x02b1dcc8; // ttgo-bme680-2

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;
char TTN_response[64];

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32}  // Pins for the Heltec ESP32 Lora board/ TTGO Lora32 with 3D metal antenna
};

void GetGasReference(){
  // Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
  Serial.println("Getting a new gas reference value");
  int readings = 10;
  for (int i = 1; i <= readings; i++){ // read gas for 10 x 0.150mS = 1.5secs
    gas_reference += bme.readGas();
  }
  gas_reference = gas_reference / readings;
}

String CalculateIAQ(float score){
  String IAQ_text = "Air quality:";
  score = (100-score)*5;
  if      (score >= 301)                  IAQ_text += "Hazardous";
  else if (score >= 201 && score <= 300 ) IAQ_text += "Very Unhealthy";
  else if (score >= 176 && score <= 200 ) IAQ_text += "Unhealthy+";
  else if (score >= 151 && score <= 175 ) IAQ_text += "Unhealthy-";
  else if (score >=  51 && score <= 150 ) IAQ_text += "Moderate";
  else if (score >=  00 && score <=  50 ) IAQ_text += "Good";
  return IAQ_text;
}

void do_send(osjob_t* j)
  {
    if (! bme.performReading()) {
      Serial.println("Failed to perform reading :(");
      return;
    }

    float tt = bme.temperature;
    float hh = bme.humidity;
    float pr = (bme.pressure / 100.0);
    float gr = (bme.gas_resistance / 1000.0);
    float al = bme.readAltitude(SEALEVELPRESSURE_HPA);

    Serial.print("Temperature = ");
    Serial.print(tt);
    Serial.println(" C");

    Serial.print("Pressure = ");
    Serial.print(pr);
    Serial.println(" hPa");

    Serial.print("Humidity = ");
    Serial.print(hh);
    Serial.println(" %");

    Serial.print("VOC = ");
    Serial.print(gr);
    Serial.println(" KOhms");

    Serial.print("Approx. Altitude = ");
    Serial.print(al);
    Serial.println(" m");

    Serial.println();

    //Calculate humidity contribution to IAQ index
    float current_humidity = bme.readHumidity();
    if (current_humidity >= 38 && current_humidity <= 42)
      hum_score = 0.25*100; // Humidity +/-5% around optimum
    else
    { //sub-optimal
      if (current_humidity < 38)
        hum_score = 0.25/hum_reference*current_humidity*100;
      else
      {
        hum_score = ((-0.25/(100-hum_reference)*current_humidity)+0.416666)*100;
      }
    }

    //Calculate gas contribution to IAQ index
    float gas_lower_limit = 5000;   // Bad air quality limit
    float gas_upper_limit = 50000;  // Good air quality limit
    if (gas_reference > gas_upper_limit) gas_reference = gas_upper_limit;
    if (gas_reference < gas_lower_limit) gas_reference = gas_lower_limit;
    gas_score = (0.75/(gas_upper_limit-gas_lower_limit)*gas_reference -(gas_lower_limit*(0.75/(gas_upper_limit-gas_lower_limit))))*100;

    //Combine results for the final IAQ index value (0-100% where 100% is good quality air)
    float air_quality_score = hum_score + gas_score;

    Serial.println();
    Serial.println("Starting new cycle...");
    Serial.println("Air Quality = "+String(air_quality_score,1)+"% derived from 25% of Humidity reading and 75% of Gas reading - 100% is good quality air");
    Serial.println("Humidity element was : "+String(hum_score/100)+" of 0.25");
    Serial.println("     Gas element was : "+String(gas_score/100)+" of 0.75");
    if (bme.readGas() < 120000) Serial.println("***** Poor air quality *****");
    Serial.println();
    if ((getgasreference_count++)%10==0) GetGasReference();
    Serial.println(CalculateIAQ(air_quality_score));
    Serial.println("------------------------------------------------");

    // PM Sensor
    PM25_AQI_Data data;
    if (! aqi.read(&data)) {
      Serial.println("Could not read from AQI");
      delay(500);  // try again in a bit!
      return;
    }
    Serial.println("AQI reading success");
    Serial.println();
    Serial.println(F("---------------------------------------"));
    Serial.println(F("Concentration Units (standard)"));
    Serial.println(F("---------------------------------------"));
    Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_standard);
    Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_standard);
    Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_standard);
    Serial.println(F("Concentration Units (environmental)"));
    Serial.println(F("---------------------------------------"));
    Serial.print(F("PM 1.0: ")); Serial.print(data.pm10_env);
    Serial.print(F("\t\tPM 2.5: ")); Serial.print(data.pm25_env);
    Serial.print(F("\t\tPM 10: ")); Serial.println(data.pm100_env);
    Serial.println(F("---------------------------------------"));
    Serial.print(F("Particles > 0.3um / 0.1L air:")); Serial.println(data.particles_03um);
    Serial.print(F("Particles > 0.5um / 0.1L air:")); Serial.println(data.particles_05um);
    Serial.print(F("Particles > 1.0um / 0.1L air:")); Serial.println(data.particles_10um);
    Serial.print(F("Particles > 2.5um / 0.1L air:")); Serial.println(data.particles_25um);
    Serial.print(F("Particles > 5.0um / 0.1L air:")); Serial.println(data.particles_50um);
    Serial.print(F("Particles > 10 um / 0.1L air:")); Serial.println(data.particles_100um);
    Serial.println(F("---------------------------------------"));

    // Starting the Lorawan payload
    uint8_t payload[28];
    uint16_t payload_size = sizeof(payload);

    uint16_t tt_16 = tt * 100;
    uint16_t hh_16 = hh * 100;
    uint32_t pr_32 = pr * 100;
    uint32_t gr_32 = gr * 100;
    uint32_t al_32 = al * 100;
    uint32_t pm10_32 = data.pm10_standard;
    uint32_t pm25_32 = data.pm25_standard;
    uint32_t pm100_32 = data.pm100_standard;

    payload[0] = tt_16 >> 8;
    payload[1] = tt_16;
    payload[2] = hh_16 >> 8;
    payload[3] = hh_16;
    payload[4] = pr_32 >> 24;
    payload[5] = pr_32 >> 16;
    payload[6] = pr_32 >> 8;
    payload[7] = pr_32;
    payload[8] = gr_32 >> 24;
    payload[9] = gr_32 >> 16;
    payload[10] = gr_32 >> 8;
    payload[11] = gr_32;
    payload[12] = al_32 >> 24;
    payload[13] = al_32 >> 16;
    payload[14] = al_32 >> 8;
    payload[15] = al_32;
    payload[16] = pm10_32 >> 24;
    payload[17] = pm10_32 >> 16;
    payload[18] = pm10_32 >> 8;
    payload[19] = pm10_32;
    payload[20] = pm25_32 >> 24;
    payload[21] = pm25_32 >> 16;
    payload[22] = pm25_32 >> 8;
    payload[23] = pm25_32;
    payload[24] = pm100_32 >> 24;
    payload[25] = pm100_32 >> 16;
    payload[26] = pm100_32 >> 8;
    payload[27] = pm100_32;

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, payload_size, 1);
        Serial.print(F("Sending uplink packet..."));
        Serial.println(counter);
        digitalWrite(LEDPIN, HIGH);
        display.clear();
        display.drawString (0, 0, "Sending uplink...");
        display.drawString (0, 10, "Temp:");
        display.drawString (35, 10, String(tt));
        display.drawString (65, 10, "°C");
        display.drawString (0, 20, "Humi:");
        display.drawString (35, 20, String(hh));
        display.drawString (65, 20, "%");
        display.drawString (0, 30, "AQI:");
        display.drawString (35, 30, String(air_quality_score,1));
        display.drawString (65, 30, "%");
        display.drawString (0, 40, String(CalculateIAQ(air_quality_score)));
        display.drawString (0, 50, String (counter));
        display.display ();
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void onEvent (ev_t ev) {
    if (ev == EV_TXCOMPLETE) {
        display.drawString (85, 0, "Done");
        Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
        if (LMIC.txrxFlags & TXRX_ACK) {
          Serial.println(F("Received ack"));
          //display.drawString (0, 20, "Received ACK.");
        }

        if (LMIC.dataLen) {
          int i = 0;
          // data received in rx slot after tx
          Serial.print(F("Data Received: "));
          Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
          Serial.println();
          //display.drawString (0, 20, "Received DATA.");

          for ( i = 0 ; i < LMIC.dataLen ; i++ )
            TTN_response[i] = LMIC.frame[LMIC.dataBeg+i];
          TTN_response[i] = 0;
          //display.drawString (0, 32, String(TTN_response));
        }

        // Schedule next transmission
        os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
        digitalWrite(LEDPIN, LOW);
        display.drawString (0, 50, String (counter));
        display.display();
        counter++;

        //Set timer to sleep TIME_TO_SLEEP seconds
        //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
        //Serial.println("Put ESP32 to sleep for " + String(TIME_TO_SLEEP) + " Seconds");

        //Go to sleep now
        //esp_deep_sleep_start();
        sleep(20);
    }
}

int getChipRevision()
{
  return (REG_READ(EFUSE_BLK0_RDATA3_REG) >> (EFUSE_RD_CHIP_VER_REV1_S)&&EFUSE_RD_CHIP_VER_REV1_V) ;
}

void printESPRevision() {
  Serial.print("REG_READ(EFUSE_BLK0_RDATA3_REG): ");
  Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG), BIN);

  Serial.print("EFUSE_RD_CHIP_VER_REV1_S: ");
  Serial.println(EFUSE_RD_CHIP_VER_REV1_S, BIN);

  Serial.print("EFUSE_RD_CHIP_VER_REV1_V: ");
  Serial.println(EFUSE_RD_CHIP_VER_REV1_V, BIN);

  Serial.println();

  Serial.print("Chip Revision (official version): ");
  Serial.println(getChipRevision());

  Serial.print("Chip Revision from shift Operation ");
  Serial.println(REG_READ(EFUSE_BLK0_RDATA3_REG) >> 15, BIN);

}

void setup() {
  Serial.begin(115200);
  delay(1500);   // Give time for the serial monitor to start up
  Serial.println(F("Starting..."));

  // printESPRevision();

  // Use the Blue pin to signal transmission.
  pinMode(LEDPIN,OUTPUT);

  // reset the OLED
  pinMode(OLED_RESET,OUTPUT);
  digitalWrite(OLED_RESET, LOW);
  delay(50);
  digitalWrite(OLED_RESET, HIGH);

  display.init ();
  display.flipScreenVertically ();
  display.setFont (ArialMT_Plain_10);

  display.setTextAlignment (TEXT_ALIGN_LEFT);

  display.drawString (0, 0, "Init...");
  display.display ();

  I2Cone.begin(SDA, SCL, 100000);

  bool status2 = aqi.begin_I2C(&I2Cone);
  if (!status2) {     
    Serial.println("Could not find PM 2.5 sensor!");
    while (1) delay(10);
  }
  Serial.println("Found PM2.5 sensor.");

  bool status1 = bme.begin();
  if (!status1) {
  Serial.println("Could not find a valid BME680 sensor, check wiring!");
  while (1);
  }
  Serial.println("Found BME680 sensor.");

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // Now run the sensor for a burn-in period, then use combination of relative humidity and gas resistance to estimate indoor air quality as a percentage.
  GetGasReference();

  // LMIC init
  SPI.begin(5, 19, 27, 18);
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);

  // Set static session parameters.
  LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);

  // Disable link check validation
  LMIC_setLinkCheckMode(0);

  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;

  // Define sub-band
  LMIC_selectSubBand(1);

  // Define the single channel and data rate (SF) to use
  //int channel = 16;
  int dr = DR_SF9;

  // Disable all channels, except for the one defined above.
  // FOR TESTING ONLY!
  //for(int i=0; i<71; i++) { // For EU; for US use i<71
  //  if(i != channel) {
  //    LMIC_disableChannel(i);
  //  }
  //}

  // Set data rate (SF) and transmit power for uplink
  LMIC_setDrTxpow(dr, 17);

  // Start job
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}