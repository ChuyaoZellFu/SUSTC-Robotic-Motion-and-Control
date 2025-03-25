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

class UARTPortId {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type UARTPortId
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type UARTPortId
    let len;
    let data = new UARTPortId(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/UARTPortId';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9298e5854915673894eb15fed41f2130';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UART_PORT_UNSPECIFIED = 0
    
    uint32 UART_PORT_EXPANSION = 1
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new UARTPortId(null);
    return resolved;
    }
};

// Constants for message
UARTPortId.Constants = {
  UART_PORT_UNSPECIFIED: 0,
  UART_PORT_EXPANSION: 1,
}

module.exports = UARTPortId;
