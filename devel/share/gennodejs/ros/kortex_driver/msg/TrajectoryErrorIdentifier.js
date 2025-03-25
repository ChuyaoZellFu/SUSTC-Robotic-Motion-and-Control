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

class TrajectoryErrorIdentifier {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrajectoryErrorIdentifier
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrajectoryErrorIdentifier
    let len;
    let data = new TrajectoryErrorIdentifier(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/TrajectoryErrorIdentifier';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd64304d18e3e5294df54e8c5ef2c83c9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 TRAJECTORY_ERROR_IDENTIFIER_UNSPECIFIED = 0
    
    uint32 TRAJECTORY_ERROR_IDENTIFIER_UNAPPLICABLE = 1
    
    uint32 TRAJECTORY_ERROR_IDENTIFIER_TIME = 2
    
    uint32 TRAJECTORY_ERROR_IDENTIFIER_POSITION = 3
    
    uint32 TRAJECTORY_ERROR_IDENTIFIER_VELOCITY = 4
    
    uint32 TRAJECTORY_ERROR_IDENTIFIER_ACCELERATION = 5
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrajectoryErrorIdentifier(null);
    return resolved;
    }
};

// Constants for message
TrajectoryErrorIdentifier.Constants = {
  TRAJECTORY_ERROR_IDENTIFIER_UNSPECIFIED: 0,
  TRAJECTORY_ERROR_IDENTIFIER_UNAPPLICABLE: 1,
  TRAJECTORY_ERROR_IDENTIFIER_TIME: 2,
  TRAJECTORY_ERROR_IDENTIFIER_POSITION: 3,
  TRAJECTORY_ERROR_IDENTIFIER_VELOCITY: 4,
  TRAJECTORY_ERROR_IDENTIFIER_ACCELERATION: 5,
}

module.exports = TrajectoryErrorIdentifier;
