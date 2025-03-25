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

class RobotiqGripperStatusFlags {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotiqGripperStatusFlags
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotiqGripperStatusFlags
    let len;
    let data = new RobotiqGripperStatusFlags(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/RobotiqGripperStatusFlags';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '289d3fd6bd510e8e9380318bbf77ab70';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_ROBOTIQ_STATUS = 0
    
    uint32 ROBOTIQ_STAT_INITIALIZED = 1
    
    uint32 ROBOTIQ_STAT_OBJECT_DETECTED = 2
    
    uint32 ROBOTIQ_STAT_POS_REACHED = 4
    
    uint32 ROBOTIQ_STAT_STOPPED = 8
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotiqGripperStatusFlags(null);
    return resolved;
    }
};

// Constants for message
RobotiqGripperStatusFlags.Constants = {
  UNSPECIFIED_ROBOTIQ_STATUS: 0,
  ROBOTIQ_STAT_INITIALIZED: 1,
  ROBOTIQ_STAT_OBJECT_DETECTED: 2,
  ROBOTIQ_STAT_POS_REACHED: 4,
  ROBOTIQ_STAT_STOPPED: 8,
}

module.exports = RobotiqGripperStatusFlags;
