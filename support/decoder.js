// Decode decodes an array of bytes into an object.
//  - fPort contains the LoRaWAN fPort number
//  - bytes is an array of bytes, e.g. [225, 230, 255, 0]
// The function must return an object, e.g. {"temperature": 22.5}
function Decode(fPort, bytes) {
  
    var decoded = {};
    
    // Temperature on bytes 0 and 1
    var celciusInt = (bytes[0] & 0x80 ? 0xFFFF<<16 : 0) | bytes[0]<<8 | bytes[1];
    decoded.tempC = celciusInt / 100;
    // Humidity on bytes 2 and 3
    var humiInt = (bytes[2] & 0x80 ? 0xFFFF<<16 : 0) | bytes[2]<<8 | bytes[3];
    decoded.humidity = humiInt / 100;
    // Pressure on bytes 4, 5, 6 and 7
    var pressInt = (bytes[4] & 0x80 ? 0xFFFF<<32 : 0) | bytes[4]<<24 | bytes[5]<<16 | bytes[6]<<8 | bytes[7];
    decoded.pressure = pressInt / 100;
    // VOC on bytes 8, 9, 10 and 11
    var VOC = (bytes[8] & 0x80 ? 0xFFFF<<32 : 0) | bytes[8]<<24 | bytes[9]<<16 | bytes[10]<<8 | bytes[11];
    decoded.VOC = VOC / 100;
    // Altitude on bytes 12, 13, 14 and 15
    var Alt = (bytes[12] & 0x80 ? 0xFFFF<<32 : 0) | bytes[12]<<24 | bytes[13]<<16 | bytes[14]<<8 | bytes[15];
    decoded.Altitude = Alt / 100;
    // PM 1.0 on bytes 16, 17, 18 and 19
    var Pm10 = (bytes[16] & 0x80 ? 0xFFFF<<32 : 0) | bytes[16]<<24 | bytes[17]<<16 | bytes[18]<<8 | bytes[19];
    decoded.PM1_0 = Pm10;
    // PM 2.5 on bytes 20, 21, 22 and 23
    var Pm25 = (bytes[20] & 0x80 ? 0xFFFF<<32 : 0) | bytes[20]<<24 | bytes[21]<<16 | bytes[22]<<8 | bytes[23];
    decoded.PM2_5 = Pm25;
    // PM 10.0 on bytes 24, 25, 26 and 27
    var Pm100 = (bytes[24] & 0x80 ? 0xFFFF<<32 : 0) | bytes[24]<<24 | bytes[25]<<16 | bytes[26]<<8 | bytes[27];
    decoded.PM10_0 = Pm100;
    
    // Return the array
    return decoded;
  }