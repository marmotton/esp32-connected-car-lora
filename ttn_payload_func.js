function Decoder(bytes, port) {
  var decoded = {};

  if ( port == 1 && bytes.length == 2 ) {
    decoded.isLocked = ( bytes[0] >> 3 ) & 1;
    decoded.isCharging = bytes[0] & 1;

    decoded.batteryKWH = Math.round( 10 * ( 30 * bytes[1] / 255) ) / 10;
  }

  if ( port == 1 && bytes.length == 11 ) {
    decoded.isLocked = ( bytes[0] >> 3 ) & 1;
    decoded.isCharging = bytes[0] & 1;

    decoded.batteryKWH = Math.round( 10 * ( 30 * bytes[1] / 255) ) / 10;

    decoded.latitude  = Math.round( 10000 * ( ( bytes[2] + 256 * bytes[3] + 256 * 256 * bytes[4] ) / 0xFFFFFF * 180  -  90 ) ) / 10000;
    decoded.longitude = Math.round( 10000 * ( ( bytes[5] + 256 * bytes[6] + 256 * 256 * bytes[7] ) / 0xFFFFFF * 360  - 180 ) ) / 10000;
    decoded.altitude  = Math.round(    10 * ( ( bytes[8] + 256 * bytes[9]                        ) / 0xFFFF   * 5200 - 200 ) ) / 10;

    decoded.hdop =  bytes[10] / 10;
  }

    if ( port == 2 && bytes.length == 9 ) {

    decoded.latitude  = Math.round( 10000 * ( ( bytes[0] + 256 * bytes[1] + 256 * 256 * bytes[2] ) / 0xFFFFFF * 180  -  90 ) ) / 10000;
    decoded.longitude = Math.round( 10000 * ( ( bytes[3] + 256 * bytes[4] + 256 * 256 * bytes[5] ) / 0xFFFFFF * 360  - 180 ) ) / 10000;
    decoded.altitude  = Math.round(    10 * ( ( bytes[6] + 256 * bytes[7]                        ) / 0xFFFF   * 5200 - 200 ) ) / 10;

    decoded.hdop =  bytes[8] / 10;
  }

  return decoded;
}
