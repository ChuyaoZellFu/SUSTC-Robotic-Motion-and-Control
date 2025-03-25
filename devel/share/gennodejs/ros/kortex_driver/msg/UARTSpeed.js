// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class UARTSpeed {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UARTSpeed
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UARTSpeed
    let len;
    let data = new UARTSpeed(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/UARTSpeed';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '75fb3218a247fa4c93b3a9a066077af5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UART_SPEED_UNSPECIFIED = 0
    
    uint32 UART_SPEED_4800 = 1
    
    uint32 UART_SPEED_9600 = 2
    
    uint32 UART_SPEED_19200 = 3
    
    uint32 UART_SPEED_38400 = 4
    
    uint32 UART_SPEED_57600 = 5
    
    uint32 UART_SPEED_115200 = 6
    
    uint32 UART_SPEED_230400 = 7
    
    uint32 UART_SPEED_460800 = 8
    
    uint32 UART_SPEED_921600 = 9
    
    uint32 UART_SPEED_1382400 = 10
    
    uint32 UART_SPEED_1612800 = 11
    
    uint32 UART_SPEED_1843200 = 12
    
    uint32 UART_SPEED_2073600 = 13
    
    uint32 UART_SPEED_2188800 = 14
    
    uint32 UART_SPEED_2246400 = 15
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UARTSpeed(null);
    return resolved;
    }
};

// Constants for message
UARTSpeed.Constants = {
  UART_SPEED_UNSPECIFIED: 0,
  UART_SPEED_4800: 1,
  UART_SPEED_9600: 2,
  UART_SPEED_19200: 3,
  UART_SPEED_38400: 4,
  UART_SPEED_57600: 5,
  UART_SPEED_115200: 6,
  UART_SPEED_230400: 7,
  UART_SPEED_460800: 8,
  UART_SPEED_921600: 9,
  UART_SPEED_1382400: 10,
  UART_SPEED_1612800: 11,
  UART_SPEED_1843200: 12,
  UART_SPEED_2073600: 13,
  UART_SPEED_2188800: 14,
  UART_SPEED_2246400: 15,
}

module.exports = UARTSpeed;
