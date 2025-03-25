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

class ServoingMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ServoingMode
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ServoingMode
    let len;
    let data = new ServoingMode(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ServoingMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3ce819b6b24c3de6343a9b4a6de953eb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_SERVOING_MODE = 0
    
    uint32 SINGLE_LEVEL_SERVOING = 2
    
    uint32 LOW_LEVEL_SERVOING = 3
    
    uint32 BYPASS_SERVOING = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ServoingMode(null);
    return resolved;
    }
};

// Constants for message
ServoingMode.Constants = {
  UNSPECIFIED_SERVOING_MODE: 0,
  SINGLE_LEVEL_SERVOING: 2,
  LOW_LEVEL_SERVOING: 3,
  BYPASS_SERVOING: 4,
}

module.exports = ServoingMode;
