#include <Arduino.h>
#include "Chassis.h"
#include "Wheel.h"
#include "PinAssignments.h"

#define START_MARKER '<'
#define END_MARKER '>'
#define COMMAND_SEP '|'
#define VALUE_SEP ','

// HARDWARE PINS
const int right_signal_pin = A0;
const int left_signal_pin = A1;

void move_motor(char *direction, int speed);
bool isWhiteSpace(char character);
int parseInt(char *input);
void printArrayOfInts();



Wheel wheels[] = {
  Wheel(FL_PWM, FL_DIR1, FL_DIR2, true), //fl
  Wheel(FR_PWM, FR_DIR1, FR_DIR2, true), //fr
  Wheel(BL_PWM, BL_DIR1, BL_DIR2, true), //bl
  Wheel(BR_PWM, BR_DIR1, BR_DIR2, true, 0.93) //br
};

Chassis chassis = Chassis(
  wheels[0], // front left
  wheels[1], // front right
  wheels[2], // back left
  wheels[3]  // back right
);

// variables updated in loop
int right_intensity = 0;
int left_intensity = 0;
int speed_gain = 0;
int motor_speed = 0;
int motorspeeddiff = 0;
int leftmotorspeed = 0;

int rightmotorspeed = 0;


int thresh = 200; // off the white tape

void setup() {

  Serial.begin(115200);
  chassis.init();
  Serial.println("Chassis initialized");
  chassis.stop();

}

const size_t buffLen = 6;     // length of the expected message chunks (number of characters between two commas) (16-bit int has 5 digits + sign)
char buffer[buffLen + 1];     // add one for terminating null character
const size_t cmdBuffLen = 10; // length of the expected command string
char cmdBuffer[cmdBuffLen + 1];

uint8_t bufferIndex = 0;

const size_t arrayOfIntsLen = 16; // number of ints to receive
int arrayOfInts[arrayOfIntsLen];
uint8_t arrayOfIntsIndex = 0;

bool receiving = false;       // set to true when start marker is received, set to false when end marker is received
bool commandReceived = false; // set to true when command separator is received (or if command buffer is full)

void loop() {
    if (Serial.available() > 0)
    {                                    // If there's at least one byte to read
        char serialByte = Serial.read(); // Read it

        if (isWhiteSpace(serialByte))
            return; // Ignore whitespace

        if (serialByte == START_MARKER)
        { // Start marker received: reset indices and flags
            receiving = true;
            commandReceived = false;
            bufferIndex = 0;
            arrayOfIntsIndex = 0;
            return;
        }
        if (receiving)
        { // If the start marker has been received
            if (!commandReceived)
            { // If the command hasn't been received yet
                if (serialByte == COMMAND_SEP || serialByte == END_MARKER)
                {                                  // If the command separator is received
                    cmdBuffer[bufferIndex] = '\0'; // Terminate the string in the buffer
                    if (strcmp(cmdBuffer, "DRIVE") == 0 || strcmp(cmdBuffer, "STOP") == 0) 
                    { 
                        Serial.print("Command received: ");
                        Serial.println(cmdBuffer);
                        // commandReceived = true;
                    }
                    else
                    {
                        Serial.print("Unknown command (");
                        Serial.print(cmdBuffer);
                        Serial.println("):");
                        // receiving = false; // Stop receiving
                    }
                    
                    if (serialByte == END_MARKER)
                    { // If the end marker is received
                        Serial.println("Message finished: (No data)");
                        Serial.println();
                        receiving = false; // Stop receivinng
                    }
                    else
                    {
                        bufferIndex = 0; // Reset the index of the buffer to overwrite it with the numbers we're about to receive
                        commandReceived = true;
                    }
                }
                else if (bufferIndex < cmdBuffLen)
                {                                          // If the received byte is not the command separator or the end marker and the command buffer is not full
                    cmdBuffer[bufferIndex++] = serialByte; // Write the new data into the buffer
                }
                else
                { // If the command buffer is full
                    Serial.println("Error: command buffer full, command is truncated");
                }
            }
            else if (serialByte == VALUE_SEP || serialByte == END_MARKER)
            { // If the value separator or the end marker is received
                if (bufferIndex == 0)
                { // If the buffer is still empty
                    Serial.println("\t(Empty input)");
                }
                else
                {                               // If there's data in the buffer and the value separator or end marker is received
                    buffer[bufferIndex] = '\0'; // Terminate the string
                    arrayOfInts[arrayOfIntsIndex++] = parseInt(buffer);   // Parse the input
                    bufferIndex = 0;            // Reset the index of the buffer to overwrite it with the next number
                }
                if (serialByte == END_MARKER)
                { // If the end marker is received
                    Serial.println("Message finished:");
                    printArrayOfInts(); // Print the values
                    Serial.println();
                    receiving = false; // Stop receivinng
                }
            }
            else if (bufferIndex < buffLen)
            {                                       // If the received byte is not a special character and the buffer is not full yet
                buffer[bufferIndex++] = serialByte; // Write the new data into the buffer
            }
            else
            { // If the buffer is full
                Serial.println("Error: buffer is full, data is truncated");
            }
            
            if(strcmp(cmdBuffer, "DRIVE") == 0 && arrayOfIntsIndex == 3){
                chassis.angleDriveRotate(arrayOfInts[0], arrayOfInts[1], arrayOfInts[2]);
            }
            else if(strcmp(cmdBuffer, "STOP") == 0){
                chassis.stop();
            }

            return; // Optional (check for next byte before executing the loop, may prevent the RX buffer from overflowing)
        }           // end if (receiving)
    
    }               // end if (Serial.available() > 0)
  
}


bool isWhiteSpace(char character)
{
    if (character == ' ')
        return true;
    if (character == '\r')
        return true;
    if (character == '\n')
        return true;
    return false;
}

int parseInt(char *input)
{
    Serial.print("\tInput:\t");
    Serial.println(input);
    if (arrayOfIntsIndex >= arrayOfIntsLen)
    {
        Serial.println("Error: array of ints is full");
        return 0;
    }
    int value = atoi(input);
    return value;
}

void printArrayOfInts()
{
    for (uint8_t i = 0; i < arrayOfIntsIndex; i++)
    {
        Serial.print(arrayOfInts[i]);
        Serial.print(' ');
    }
    Serial.println();
}

void move_motor(char *direction, int speed){
    return;
}