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

class UARTStopBits {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UARTStopBits
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UARTStopBits
    let len;
    let data = new UARTStopBits(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/UARTStopBits';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '41247c97dd0da3882638e6dea902eead';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UART_STOP_BITS_UNSPECIFIED = 0
    
    uint32 UART_STOP_BITS_0_5 = 1
    
    uint32 UART_STOP_BITS_1 = 2
    
    uint32 UART_STOP_BITS_1_5 = 3
    
    uint32 UART_STOP_BITS_2 = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UARTStopBits(null);
    return resolved;
    }
};

// Constants for message
UARTStopBits.Constants = {
  UART_STOP_BITS_UNSPECIFIED: 0,
  UART_STOP_BITS_0_5: 1,
  UART_STOP_BITS_1: 2,
  UART_STOP_BITS_1_5: 3,
  UART_STOP_BITS_2: 4,
}

module.exports = UARTStopBits;
