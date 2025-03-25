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

class UARTParity {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UARTParity
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UARTParity
    let len;
    let data = new UARTParity(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/UARTParity';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '02ea0fa72b95d9977ccc489459fb1109';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UART_PARITY_UNSPECIFIED = 0
    
    uint32 UART_PARITY_NONE = 1
    
    uint32 UART_PARITY_ODD = 2
    
    uint32 UART_PARITY_EVEN = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UARTParity(null);
    return resolved;
    }
};

// Constants for message
UARTParity.Constants = {
  UART_PARITY_UNSPECIFIED: 0,
  UART_PARITY_NONE: 1,
  UART_PARITY_ODD: 2,
  UART_PARITY_EVEN: 3,
}

module.exports = UARTParity;
