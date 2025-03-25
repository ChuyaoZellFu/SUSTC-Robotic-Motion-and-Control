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

class OperatingMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OperatingMode
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OperatingMode
    let len;
    let data = new OperatingMode(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/OperatingMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5399510deee390e03ac867dbfd636f27';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_OPERATING_MODE = 0
    
    uint32 MAINTENANCE_MODE = 1
    
    uint32 UPDATE_MODE = 2
    
    uint32 UPDATE_COMPLETED_MODE = 3
    
    uint32 UPDATE_FAILED_MODE = 4
    
    uint32 SHUTTING_DOWN_MODE = 5
    
    uint32 RUN_MODE = 6
    
    uint32 UPDATING_DEVICE_MODE = 7
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OperatingMode(null);
    return resolved;
    }
};

// Constants for message
OperatingMode.Constants = {
  UNSPECIFIED_OPERATING_MODE: 0,
  MAINTENANCE_MODE: 1,
  UPDATE_MODE: 2,
  UPDATE_COMPLETED_MODE: 3,
  UPDATE_FAILED_MODE: 4,
  SHUTTING_DOWN_MODE: 5,
  RUN_MODE: 6,
  UPDATING_DEVICE_MODE: 7,
}

module.exports = OperatingMode;
