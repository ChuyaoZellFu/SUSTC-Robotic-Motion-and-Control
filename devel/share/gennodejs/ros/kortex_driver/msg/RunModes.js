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

class RunModes {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RunModes
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RunModes
    let len;
    let data = new RunModes(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/RunModes';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4d935d7a06871eb332098b80bd9adf59';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 RUN_MODE = 0
    
    uint32 CALIBRATION_MODE = 1
    
    uint32 CONFIGURATION_MODE = 2
    
    uint32 DEBUG_MODE = 3
    
    uint32 TUNING_MODE = 4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RunModes(null);
    return resolved;
    }
};

// Constants for message
RunModes.Constants = {
  RUN_MODE: 0,
  CALIBRATION_MODE: 1,
  CONFIGURATION_MODE: 2,
  DEBUG_MODE: 3,
  TUNING_MODE: 4,
}

module.exports = RunModes;
