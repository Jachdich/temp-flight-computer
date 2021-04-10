#include "Arduino.h"
#include <stdint.h>
#include "RF24.h"
#include "PWMServo.h"
#include "Adafruit_BNO055.h"
#include "Adafruit_ADS1X15.h"
#include "float3.h"

typedef struct {
    int8_t elevator;
    int8_t rudder;
    int8_t aileron;
    uint8_t throttle;
} ControlPacket;

typedef struct {
    float3 orient;
    float3 angVel;
    float3 linAccel;
    float3 grav;
    uint8_t temp;
    uint16_t rail_5v0;
    uint16_t rail_8v4;
    uint16_t current_motor;
    uint16_t current_5v0;
    uint16_t staticPressure;
    uint16_t differentialPressure;
} UnpackedTelemetryPacket;

RF24 radio(9, 10);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_ADS1115 ads;
uint8_t droppedRx = 0;
PWMServo elevator;
PWMServo rudder;
PWMServo aileron;
PWMServo throttle;

void setControls(ControlPacket pack) {
    elevator.write(pack.elevator);
    rudder.write(pack.rudder);
    aileron.write(pack.aileron);
    throttle.write(pack.throttle);
}

void panic() {
    //set controls to zero, throttle down motor
    //in case of loss of communication
    
    setControls({0, 0, 0, 0});
}

float3 toFloat3(sensors_vec_t e) {
    return float3{e.x, e.y, e.z};
}

UnpackedTelemetryPacket getTelemetry() {
    sensors_event_t orient, angVel , linAccel, grav;
    bno.getEvent(&orient, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVel, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linAccel, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&grav, Adafruit_BNO055::VECTOR_GRAVITY);
    uint8_t temp = bno.getTemp();
    
    return {
        toFloat3(orient.orientation),
        toFloat3(angVel.gyro),
        toFloat3(linAccel.acceleration),
        toFloat3(grav.gyro),
        temp,
        (uint16_t)analogRead(A1),
        (uint16_t)analogRead(A2),
        (uint16_t)ads.readADC_Differential_2_3(),
        (uint16_t)ads.readADC_Differential_0_1(),
        (uint16_t)analogRead(A8),
        (uint16_t)analogRead(A9),
    };
    
}

extern "C" int main(void) {
    Serial.begin(9600);
    while (!Serial);
    if (!bno.begin()) {
        Serial.println("Failed to initialise BNO055");
        while (1);
    } else {
        Serial.println("BNO055 initialised");
    }

    if (!radio.begin()) {
        Serial.println("Radio failed to initialise");
        //what
        return 1;
        //lol this doesnt make sense
        //therefore make the µc do something that doesnt make sense either
        //lmfao get trolololed 
    } else {
        Serial.println("Radio initialised\n");
    }

    ads.setGain(GAIN_SIXTEEN);
    ads.begin();
    
    radio.setPayloadSize(32);

    radio.openWritingPipe((const unsigned char*)"B0001");
    radio.openReadingPipe(1, (const unsigned char*)"A0001");
    radio.setPALevel(RF24_PA_MAX); //TODO MAX in real code lel
    radio.setDataRate(RF24_250KBPS);

    while (1) {
        UnpackedTelemetryPacket pack = getTelemetry();

        uint8_t* packet_bytes = (uint8_t*)&pack;

        radio.stopListening();
        for (uint8_t packet_idx = 0; packet_idx < 2; packet_idx++) {
            radio.write(packet_bytes + (packet_idx * 32), 32);
        }
        radio.startListening();

        uint32_t ms = millis();
        while (!radio.available()) {
            if (millis() - ms > 100) {
                //assume packet is not coming after 100ms
                droppedRx++;
                if (droppedRx > 10) {
                    panic();
                    droppedRx = 0;
                }
                break;
            }
        }

        if (radio.available()) {
            ControlPacket packet;
            uint8_t* recvBuffer = (uint8_t*)&packet;
            radio.read(recvBuffer, sizeof(ControlPacket));
            setControls(packet);
        }
    }
}
