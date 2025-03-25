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

class EthernetSpeed {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EthernetSpeed
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EthernetSpeed
    let len;
    let data = new EthernetSpeed(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/EthernetSpeed';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '03143ee942f3291a850b685ab265f926';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 ETHERNET_SPEED_UNSPECIFIED = 0
    
    uint32 ETHERNET_SPEED_10M = 1
    
    uint32 ETHERNET_SPEED_100M = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EthernetSpeed(null);
    return resolved;
    }
};

// Constants for message
EthernetSpeed.Constants = {
  ETHERNET_SPEED_UNSPECIFIED: 0,
  ETHERNET_SPEED_10M: 1,
  ETHERNET_SPEED_100M: 2,
}

module.exports = EthernetSpeed;
