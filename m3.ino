#include <bluefruit.h>
#include <uptime.h>

/* BLE services and characterisitics */
BLEDis bledis;
BLEService cps = BLEService(UUID16_SVC_CYCLING_POWER);
BLECharacteristic cpf = BLECharacteristic(UUID16_CHR_CYCLING_POWER_FEATURE);
BLECharacteristic cpm = BLECharacteristic(UUID16_CHR_CYCLING_POWER_MEASUREMENT);
BLECharacteristic cpsl = BLECharacteristic(UUID16_CHR_SENSOR_LOCATION);

/* Global variables */
uint8_t m3addr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  // the M3i advertising address
uint16_t power = 0;  // as reported by m3i
uint16_t rpm = 0;  // as reported by m3i
float crank_count = 0.0;  // crank count based on rpm and time elapsed
uint16_t crank_count_last = 0;  // last crank count notified
uint16_t crank_count_check = 0;  // crank count to notify
unsigned long ts = 0;  // milliseconds elapsed since startup to last m3i message
uint16_t ts_last = 0;  // milliseconds elapsed since startup to last crank count notice

uint16_t pc_flags = 0b0000000000100000;
uint16_t p_flags = 0b0000000000000000;
/*
 cpm_flags are the first 16 bits of the `cpm` characteristic
   b13-15: RFU
   b12: offset compensation indicator (0: false, 1: true)
   b11: accumulated energy present (0: false, 1: true)
   b10: bottom dead spot angle present (0: false, 1: true)
   b9: top dead spot angle present (0: false, 1: true)
   b8: extreme angles present (0: false, 1: true)
   b7: extreme torque magnitudes present (0: false, 1: true)
   b6: extreme force magnitudes present (0: false, 1: true)
   b5: crank revolution data present (0: false, 1: true)
   b4: wheel revolution data present (0: false, 1: true)
   b3: accumulated torque source (0: wheel based, 1: crank based)
   b2: accumulated torque present (0: false, 1: true)
   b1: pedal power balance reference (0: unknown, 1: left)
   b0: pedal power balance present (0: false, 1: true)
 */

/****
  Set up BLE services and characteristics 
 ****/
void setup() {
    Serial.begin(115200);
    // while ( !Serial ) delay(10);   // for nrf52840 with native usb


    Serial.println("Bluefruit52 duo");
    Serial.println("-----------------------\n");
    Bluefruit.begin();

    Bluefruit.setConnLedInterval(250);
    Bluefruit.Scanner.setRxCallback(scan_callback);
    Bluefruit.Scanner.useActiveScan(true);
    Bluefruit.Scanner.start(0);
    Serial.println("Scanning...");

    Bluefruit.setName("Bluefruit52 duo");

    // Set the connect/disconnect callback handlers
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

    bledis.setManufacturer("Adafruit Industries");
    bledis.setModel("Bluefruit Feather52");
    bledis.begin();

    Serial.println("Configuring CyclingPowerService");
    setupCPS();

    startAdv();
    Serial.println("\nAdvertising...");
}

void setupCPS(void){
    cps.begin();

    /*
     * Cycling power feature configuration
     * https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.cycling_power_feature.xml
     * 
     * Properties: read
     * Length: 1x32bit
     * B0:
     *   b22-31: RFU
     *   b20-21: Distributed system support (0: unspecified, 1: unsupported, 2: supported, 3: RFU)
     *   b19: Enhanced offset compesnation supported (0: false, 1: true)
     *   b18: Factory calibration date supported (0: false, 1: true)
     *   b17: Instantaneous measurment direction supported (0: false, 1: true)
     *   b16: Sensor measurement context (0: force based, 1: torque based)
     *   b15: Span length adjustment supported (0: false, 1: true)
     *   b14: Chain weight adjustment supported (0: false, 1: true)
     *   b13: Chain length adjustment suppoorted (0: false, 1: true)
     *   b12: Crank length adjustment supported (0: false, 1: true)
     *   b11: Multiple sensor locations supported (0: false, 1: true)
     *   b10: Cycling power measurment characteristic context masking supported (0: false, 1: true)
     *   b9: Offset compensation supported (0: false, 1: true)
     *   b8: Offset compensation indicator supported (0: false, 1: true)
     *   b7: Accumulated energy supported (0: false, 1: true)
     *   b6: top and bottom dead spot angles supported (0: false, 1: true)
     *   b5: extreme angles supported (0: false, 1: true)
     *   b4: extreme magnitudes supported (0: false, 1: true)
     *   b3: crank revolution data supported (0: false, 1: true)
     *   b2: wheel revolution data supported (0: false, 1: true)
     *   b1: accumulated torque supported (0: false, 1: true)
     *   b0: pedal power balance supported (0: false, 1: true)
     */
    cpf.setProperties(CHR_PROPS_READ);
    cpf.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    cpf.setFixedLen(4); // 32 bits = 4 bytes
    uint32_t cpfdata[1] = {0b00000000000000000000000000001000};
    cpf.begin();
    cpf.write(cpfdata, sizeof(cpfdata));

    /*
     * Cycling power measurment config
     * https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.cycling_power_measurement.xml
     * properties: notify
     * max length: 8 fields = 17 bytes
     * B0: 16 bit flags
     * B1: sint16 instantaneous power
     * B2: uint8 pedal power balance
     * B3: uint16 accumulated torque
     * B4: uint32 cumulative wheel revolutions
     * B5: uint16 last wheel event time
     * B6: uint16 cumulative crank revolutions
     * B7: uint16 last crank event time
     */
    cpm.setProperties(CHR_PROPS_NOTIFY);
    cpm.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    cpm.setMaxLen(8);
    cpm.setCccdWriteCallback(cccd_callback);
    cpm.begin();

    /*
     * Cycling power sensor location
     * https://www.bluetooth.com/wp-content/uploads/Sitecore-Media-Library/Gatt/Xml/Characteristics/org.bluetooth.characteristic.sensor_location.xml
     */
    cpsl.setProperties(CHR_PROPS_READ);
    cpsl.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    cpsl.setFixedLen(1);
    cpsl.begin();
    cpsl.write8(0);    // Set the characteristic to 'Other' (0)
}

/**** 
  Receiving data from m3i
 ****/
boolean check_addr(uint8_t addr[]){
    for (int n = 0; n < 6; n++) {
        if (addr[n] != m3addr[n]) {
            return false;
        }
    }
    return true;
}

boolean is_live_data(uint8_t* buffer) {
    if (buffer[4] == 0x00) return true;
    return false;
}

void parse_m3(uint8_t* buffer){
    power = (buffer[11]<<8) | buffer[10];
    rpm = (buffer[7]<<8) | buffer[6];

    // Serial.printf("Power hex %04X\n", power);
    // Serial.printf("Power int %03i\n", power);
    // Serial.printf("Cadence RPM hex %04X\n", rpm);
    // Serial.printf("Cadence RPM int %03i\n", rpm);
    // Serial.println();
}

void scan_callback(ble_gap_evt_adv_report_t* report){
    PRINT_LOCATION();
    uint8_t len = 0;
    uint8_t buffer[32];
    memset(buffer, 0, sizeof(buffer));

    /* Check for Manufacturer Specific Data */
    len = Bluefruit.Scanner.parseReportByType(report, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, buffer, sizeof(buffer));
    if (len) {
        // Serial.printf("%14s ", "MAN SPEC DATA");
        // Serial.printBuffer(buffer, len, '-');
        // Serial.println();

        /* Check if data is from M3 */
        if (check_addr(report->peer_addr.addr)){

            /* Check if data is "live" */
            if (is_live_data(buffer)){
                parse_m3(buffer);
                eval_crank_count();

                /* Notify immediately */
                if ( Bluefruit.connected() ) {
                    send_power();
                }
            }
        }
    }

    // release buffer
    memset(buffer, 0, sizeof(buffer));

    // For Softdevice v6: after received a report, scanner will be paused
    // We need to call Scanner resume() to continue scanning
    Bluefruit.Scanner.resume();

}


/****
  Calculate cadence values to broadcast
 ****/
unsigned long elapsed() {
    uptime::calculateUptime();
    return uptime::getHours() * 3600000 +
        uptime::getMinutes() * 60000 +
        uptime::getSeconds() * 1000 +
        uptime::getMilliseconds();
}

/*
   eval_crank_count() executes when rpm data is received from the m3i
   `crank_count` gets updated based on the reported rpm and the time elapsed since the last m3i message
   `ts` gets updated
 */
void eval_crank_count() {
    unsigned long interval = elapsed() - ts;
    crank_count = crank_count + ((float)interval * (float)rpm / 60000.0 / 10.0);
                                                          // extra /10.0 ^ because
                                                          // m3 rpm uses an extra order of mag
    ts = ts + interval;
    // Serial.printf("crank_count: %4f\n", crank_count);
    // Serial.printf("elapsed: %4i\n", ts);
}

uint16_t ts_delta(uint16_t rpm, uint16_t count) {
    return count * 6000000.0 / float(rpm) / 10.0;  // divide by 10 since rpm is increased order of mag
}

/*
   resolve_crank_count() executes when cpm sends a notification
   `crank_count` is converted to an integer and incremented if necessary
 */
void resolve_crank_count() {
    crank_count_check = (uint16_t) round(crank_count);

    // make sure count delta > 0, otherwise inferred rpm will be 0
    if (crank_count_check <= crank_count_last){
        crank_count_check = crank_count_last + 1;
    }

    // make sure crank_count doesn't fall behind crank_count_check
    if (crank_count < crank_count_check) {
        crank_count = crank_count_check;
    }

    // interpolate crank delta duration from the reported rpm
    ts_last = ts_last + ts_delta(rpm, crank_count_check - crank_count_last);
    
    // reset crank count for next update
    crank_count_last = crank_count_check;
}

uint16_t scale_millis(uint16_t n){
    return n * 1024 / 1000;
}

void startAdv(void){
    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();

    // Include UUID and name
    Bluefruit.Advertising.addService(cps);
    Bluefruit.Advertising.addName();

    /* Start Advertising
     * - Enable auto advertising if disconnected
     * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     * - Timeout for fast mode is 30 seconds
     * - Start(timeout) with timeout = 0 will advertise forever (until connected)
     * 
     * For recommended advertising interval
     * https://developer.apple.com/library/content/qa/qa1931/_index.html   
     */
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
    Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
    Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds 
}

void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);
  Serial.println("Advertising!");

}

void cccd_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint16_t cccd_value) {
    // Display the raw request packet
    Serial.print("CCCD Updated: ");
    //Serial.printBuffer(request->data, request->len);
    Serial.print(cccd_value);
    Serial.println("");

    // Check the characteristic this CCCD update is associated with in case
    // this handler is used for multiple CCCD records.
    if (chr->uuid == cpm.uuid) {
        if (chr->notifyEnabled(conn_hdl)) {
            Serial.println("Cycling Power Measurement 'Notify' enabled");
        } else {
            Serial.println("Cycling Power Measurement 'Notify' disabled");
        }
    }
}

void send_power() {
    resolve_crank_count();

    uint16_t cpmdata[4];
    cpmdata[0] = pc_flags;
    cpmdata[1] = power;
    cpmdata[2] = crank_count_check;
    cpmdata[3] = (uint16_t) scale_millis((uint16_t) ts_last);
    
    // Serial.printf("Sending crank revs: %3i\n", cpmdata[2]);
    // Serial.printf("Sending crank time: %9i\n", cpmdata[3]);
    // Serial.printf("elapsed: %9i\n", ts);

    if (cpm.notify(cpmdata, sizeof(cpmdata))) {
        // Serial.printf("Sent power: %03i\n", cpmdata[1]);
        // Serial.printf("Sent crank revs: %03i\n", cpmdata[2]);
        // Serial.printf("Sent crank time: %03i\n", cpmdata[3]);
    } else {
        Serial.println("ERROR: Notify not set in the CCCD or not connected!");
    }
}

void loop() {
    // eval_crank_count();
    // if ( Bluefruit.connected() ) {
    //     send_power();
    // }
    // delay(500);  // send twice per second
}
