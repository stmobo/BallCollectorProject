typedef struct PIDData {
    float setpoint;
    float i_acc;    // integral accumulator
    float last_err; // last error value
    short lastSensorValue;

    bool invertSensor;
    short sensorPort;
    short motorPort;
} PIDData;

float kP = 0.35;
float kI = 0;
float kD = 0;
float kF = -0.001;
float iZone = 2048;

PIDData leftPID;
PIDData rightPID;

rightPID.motorPort = motor1;
rightPID.sensorPort = enc1;
rightPID.invertSensor = true;

leftPID.motorPort = motor2;
leftPID.sensorPort = enc2;
leftPID.invertSensor = false;

void pidInit(PIDData *data) {
    data->setpoint = 0;
    data->i_acc = 0;
    data->last_err = 0;
    data->lastSensorValue = 0;
}

void pidUpdate(PIDData *data, float dt) {
    float val = SensorValue(data->sensorPort);
    if(data->invertSensor)
        val *= -1;

    float vel = (val - data->lastSensorValue) / dt;
    data->lastSensorValue = val;

    float err = data->setpoint - vel;

    data->i_acc += err * dt;
    if(data->i_acc > iZone || data->i_acc < -iZone) {
        data->i_acc = 0;
    }

    float d_err = (err - data->last_err) / dt;
    data->last_err = err;

    float p = err * kP;
    float i = data->i_acc * kI;
    float d = d_err * kD;
    float f = data->setpoint * kF;

    float out_f = p+i+d+f;
    short out_i = (short)out_f;

    if(out_i < -127)
        out_i = -127;

    if(out_i > 127)
        out_i = 127;

    motor[data->motorPort] = out_i;
}

task pidControl() {
    time1[timerA] = 0;

    pidInit(&leftPID);
    pidInit(&rightPID);

    /* for testing */
    leftPID.setpoint = 100;
    rightPID.setpoint = 100;

    while(true) {
        float dt = time1[timerA];
        dt /= 1000;

        pidUpdate(&leftPID, dt);
        pidUpdate(&rightPID, dt);

        time1[timerA] = 0;
        sleep(20);
    }
}
