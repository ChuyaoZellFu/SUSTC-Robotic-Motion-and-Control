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

class TrajectoryContinuityMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryContinuityMode
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryContinuityMode
    let len;
    let data = new TrajectoryContinuityMode(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/TrajectoryContinuityMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a762cc9649feca6c4672c7308715b451';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 TRAJECTORY_CONTINUITY_MODE_UNSPECIFIED = 0
    
    uint32 TRAJECTORY_CONTINUITY_MODE_POSITION = 1
    
    uint32 TRAJECTORY_CONTINUITY_MODE_SPEED = 2
    
    uint32 TRAJECTORY_CONTINUITY_MODE_ACCELERATION = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryContinuityMode(null);
    return resolved;
    }
};

// Constants for message
TrajectoryContinuityMode.Constants = {
  TRAJECTORY_CONTINUITY_MODE_UNSPECIFIED: 0,
  TRAJECTORY_CONTINUITY_MODE_POSITION: 1,
  TRAJECTORY_CONTINUITY_MODE_SPEED: 2,
  TRAJECTORY_CONTINUITY_MODE_ACCELERATION: 3,
}

module.exports = TrajectoryContinuityMode;
