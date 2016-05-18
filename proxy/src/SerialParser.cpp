#include <stdint.h>
#include <string.h>
#include <iostream>
#include <string>
#include <memory>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include "SerialParser.hpp"
#include <vector>
#include <mutex>

using namespace std;
using namespace odcore;
using namespace odcore::wrapper;

string buffer = ""; // Buffer to hold incoming bytes

vector<double> parse(char type,const string &s); // Prototype for packet parsing function

vector<double> currUS; // Buffer for latest US data
vector<double> currIR; // Buffer for latest IR data

double currDist; // Buffer for latest encoder data

// Mutex locks for the US and IR buffers
mutex irBufferMtx;
mutex usBufferMtx;

// NOTHING
void checkBuffer() {
    size_t closingDelimIndex = buffer.rfind("]"); // Find the latest closing delimiter (search starting from the end of the 'buffer' string)
    size_t openingDelimIndex = buffer.rfind("["); // Find the latest opening delimiter (search starting from the end of the 'buffer' string)

    // cout << "ClosingIndex:" << closingDelimIndex << endl;
    // cout << "OpeningIndex:" << openingDelimIndex << endl;
    // cout << "Buffer:" << buffer << endl;

    if(openingDelimIndex < closingDelimIndex){ // If the opening delimiter is before the closing delimiter
        const string packet = buffer.substr(openingDelimIndex, closingDelimIndex + 1); // Attempt to form a packet
        // cout << "Packet: " << packet << endl;
        if(packet[1] == 'U' && packet[2] == 'S'){ // If it is a US sensor data packet
            currUS = parse('u', packet); // Parse, and assign the latest US value buffer
        }
        if(packet[1] == 'I' && packet[2] == 'R'){ // If it is a IR sensor data packet
            currIR = parse('i', packet);
        }
        if(packet[1] == 'E'){  // If it is an wheel encoder data packet
            std::string val;
            val = packet.substr(3, val.size() - 2);
            currDist = std::stoi(val); // Assign current distance buffer to the value in the data packet
        }
        buffer.erase(openingDelimIndex, closingDelimIndex); // Clear data packet from buffer
    }
}

vector<double> SerialParser::getIR(){ // 'Getter' for the current IR values for use in Proxy.cpp
    usBufferMtx.lock();
    vector<double> returnBuffer = currIR;
    currIR.clear(); // Clear the IR buffer ('empty' vectors == no new values received)
    irBufferMtx.unlock();
    return returnBuffer;
}

vector<double> SerialParser::getUS(){ // 'Getter' for the current US values for use in Proxy.cpp
    usBufferMtx.lock();
    vector<double> returnBuffer = currUS;
    currUS.clear();
    usBufferMtx.unlock();
    return returnBuffer;
}

double SerialParser::getDist(){ // 'Getter' for the current distance values for use in Proxy.cpp
    return currDist;
}

bool SerialParser::irHasNewVals(){ // Function to return if the buffer has received new values (i.e. is not empty)
    bool returnBool = !currIR.empty();
    return returnBool;
}

bool SerialParser::usHasNewVals(){
    bool returnBool = !currUS.empty();
    return returnBool;
}

void SerialParser::nextString(const string &s) { // Interface function provided by SerialPort for when bytes are received over the serial connection
    buffer = buffer + s; // Append the bytes to the buffer
    checkBuffer(); // Check the buffer for whole packets
}   
    
vector<double> parse(char type,const string &s){ // Parse a whole data packet
    if(type == 'i'){  // IR Parsing protocol
        bool firstChecked = false;
        bool secondChecked = false;
        bool thirdChecked = false;

        std::string firstVal, secondVal, thirdVal;

        for (uint32_t i = 0; i < s.size(); i++){
            if(s[i] == ','){
                int next;

                for(uint32_t j = i; j < s.size(); j++){
                    if(s[j] == ',' || s[j] == ']') next = j;
                    break;
                }

                if(!firstChecked){
                    firstVal = s.substr(i + 1, next);
                    firstChecked = true;
                }

                else if(!secondChecked){
                    secondVal = s.substr(i + 1, next);
                    secondChecked = true;
                }

                else if(!thirdChecked){
                    thirdVal = s.substr(i + 1, next);
                    thirdChecked = true;
                }
            }

            if(firstChecked && secondChecked && thirdChecked) break;
        }

        vector<double> irVector(3);

        irVector[0] = std::stoi(firstVal);
        irVector[1] = std::stoi(secondVal);
        irVector[2] = std::stoi(thirdVal);

        return irVector;
    }

    if(type == 'u'){ // US parsing protocol
        bool firstChecked = false;
        bool secondChecked = false;

        std::string firstVal, secondVal;

        for (uint32_t i = 0; i < s.size(); i++){
            if(s[i] == ','){
                int next;

                for(uint32_t j = i; j < s.size(); j++){
                    if(s[j] == ',' || s[j] == ']') next = j;
                    break;
                }

                if(!firstChecked){
                    firstVal = s.substr(i + 1, next);
                    firstChecked = true;
                }

                else if(!secondChecked){
                    secondVal = s.substr(i + 1, next);
                    secondChecked = true;
                }
            }

            if(firstChecked && secondChecked) break;
        }

        vector<double> usVector(2);

        usVector[0] = std::stoi(firstVal);
        usVector[1] = std::stoi(secondVal);

        return usVector;
    }

    else{
        // cout << "Invalid data type received." << endl;
        vector<double> useless(0);
        return useless;
    }
}