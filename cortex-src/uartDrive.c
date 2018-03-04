#pragma config(UART_Usage, UART1, uartUserControl, baudRate9600, IOPins, None, None)
#pragma config(UART_Usage, UART2, uartUserControl, baudRate9600, IOPins, None, None)
#pragma config(I2C_Usage, I2C1, i2cSensors)
#pragma config(Sensor, I2C_1,  enc1,           sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Sensor, I2C_2,  enc2,           sensorQuadEncoderOnI2CPort,    , AutoAssign )
#pragma config(Motor,  port1,           motor2,        tmotorVex393_HBridge, openLoop)
#pragma config(Motor,  port10,          motor1,        tmotorVex393_HBridge, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

char waitForChar(short port) {
	while(true) {
		unsigned char c = getChar(port);
		if(c != 0xFF) {
			return c;
		}
	}
}

typedef struct PIDData {
    float setpoint;
    float i_acc;    // integral accumulator
    float last_err; // last error value
    short lastSensorValue;

    bool invertSensor;
    short sensorPort;
    short motorPort;
} PIDData;

float kP = 0.275;
float kI = 0;
float kD = -0.002;
float kF = 127.0 / 850.0;
float iZone = 2048;

float wrap_low = 0.3 * 65536 - 32768;
float wrap_high = 0.7 * 65536 - 32768;

PIDData leftPID;
PIDData rightPID;

void pidInit(PIDData *data) {
    data->setpoint = 0;
    data->i_acc = 0;
    data->last_err = 0;
    data->lastSensorValue = 0;
}

void pidUpdate(PIDData *data, float dt) {
    short val = SensorValue[data->sensorPort];
    if(data->invertSensor)
        val *= -1;

    float diff = 0;
    if(data->lastSensorValue > wrap_high && val < wrap_low) {
        // sensor value wrapped around past the upper limit
        diff = (32768 - data->lastSensorValue) + (32768 + val);
    } else if(data->lastSensorValue < wrap_low && val > wrap_high) {
        // sensor value wrapped around past the lower limit
         diff = (32768 + data->lastSensorValue) + (32768 - val);
    } else {
        diff = val - data->lastSensorValue;
    }

    float vel = diff / dt;

    data->lastSensorValue = val;

    float err = data->setpoint - vel;
    //writeDebugStreamLine("Err: %f", err);

    data->i_acc += err * dt;
    if(data->i_acc > iZone || data->i_acc < -iZone) {
        data->i_acc = 0;
    }

    float d_err = (err - data->last_err) / dt;
    data->last_err = err;

    float lp = err * kP;
    float li = data->i_acc * kI;
    float ld = d_err * kD;
    float lf = data->setpoint * kF;

    float out_f = lp+li+ld+lf;
    short out_i = (short)out_f;

    if(out_i < -127)
        out_i = -127;

    if(out_i > 127)
        out_i = 127;

    motor[data->motorPort] = out_i;
}

task pidControl() {
    pidInit(&leftPID);
    pidInit(&rightPID);

	rightPID.motorPort = motor1;
	rightPID.sensorPort = enc2;
	rightPID.invertSensor = false;

	leftPID.motorPort = motor2;
	leftPID.sensorPort = enc1;
	leftPID.invertSensor = true;

	SensorValue[enc1] = 0;
	SensorValue[enc2] = 0;

    time1[T2] = 0;

    sleep(20);

    while(true) {
        float dt = time1[T2];
        dt /= 1000;

        pidUpdate(&leftPID, dt);
        pidUpdate(&rightPID, dt);

        time1[T2] = 0;
        sleep(20);
    }
}

task main()
{
	configureSerialPort(UART1, uartUserControl);
	setBaudRate(UART1, baudRate9600);

	configureSerialPort(UART2, uartUserControl);
	setBaudRate(UART2, baudRate9600);

	int n = 0;
	writeDebugStreamLine("Clearing UART buffer...");
	while(getChar(UART1) != -1) {
		sleep(1);

		if(n % 500 == 0) {
			writeDebugStreamLine("Clearing UART buffer...");
		}

		n += 1;
	}

	startTask(pidControl);

	while(true) {
        //writeDebugStreamLine("Waiting for command...");
        while(waitForChar(UART1) != 0xAA) {
        	sleep(5);
        }

        unsigned char sub_cmd = waitForChar(UART1);
        if(sub_cmd == 0x01) {
            /* Set motors command */
            short m1 = waitForChar(UART1);
            short m2 = waitForChar(UART1);
            char checksum = waitForChar(UART1);
            short cmp = 0xAA ^ 0x01 ^ (m1 & 0xFF) ^ (m2 & 0xFF) ^ checksum;

            if(cmp == 0) {
                /* checksum okay, got a valid motor command */
                writeDebugStreamLine("Got motor command: %x / %x", m1, m2);

            	if(m1 > 0x7F) {
            		m1 = (((~m1) + 1) & 0xFF);
            		m1 *= -1;
            	}

            	if(m2 > 0x7F) {
            		m2 = ((~m2) + 1) & 0xFF;
            		m2 *= -1;
            	}

            	motor[motor1] = m1;
            	motor[motor2] = m2;

            	sendChar(UART1, 0x55);
            	sendChar(UART1, 0x55);
            } else {
            		short expected = 0xAA ^ 0x01 ^ (m1 & 0xFF) ^ (m2 & 0xFF);
                writeDebugStreamLine("Checksum failed for set motors command, got %x but expected %x", checksum, expected);
            }
        } else if(sub_cmd == 0x03) {
            unsigned char payload[8];
            unsigned char cmp = 0xAA ^ 0x03;
            for(int i=0;i<8;i++) {
            	payload[i] = waitForChar(UART1);
            	cmp ^= payload[i];
            }

            unsigned char checksum = waitForChar(UART1);

            if(cmp ^ checksum == 0) {
            	float *values = (float*)(&payload);
            	leftPID.setpoint = values[0];
            	rightPID.setpoint = values[1];

            	writeDebugStreamLine("Got PID motor control command: %f / %f", values[0], values[1]);

            	sendChar(UART1, 0x55);
            	sendChar(UART1, 0x55);
            } else {
            	writeDebugStreamLine("Got invalid checksum for PID motor data.");
            }
	   } else if(sub_cmd == 0x02) {
            char checksum = waitForChar(UART1);
            if(0xAA ^ 0x02 ^ checksum == 0) {
                /* Get encoder data */
    			short e1 = -SensorValue[enc1];
    			short e2 = SensorValue[enc2];

                char e1A = e1 & 0xFF;
                char e1B = (e1 >> 8) & 0xFF;

                char e2A = e2 & 0xFF;
                char e2B = (e2 >> 8) & 0xFF;

                char checksum = 0x55 ^ e1A ^ e1B ^ e2A ^ e2B;

    			sendChar(UART1, 0x55);

    			sendChar(UART1, e1A);
    			sendChar(UART1, e1B);

    			sendChar(UART1, e2A);
    			sendChar(UART1, e2B);

    			sendChar(UART1, checksum);
    			while(!bXmitComplete(UART1)) {}
            } else {
                writeDebugStreamLine("Checksum failed for read encoder data command.");
            }
		} else {
			writeDebugStreamLine("Got unknown command: 0x%x", sub_cmd);
		}
	}
}
