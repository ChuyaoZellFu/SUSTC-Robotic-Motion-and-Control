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

class GripperMode {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GripperMode
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GripperMode
    let len;
    let data = new GripperMode(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GripperMode';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '91237e67935408a1b67c1d3e83f53d6b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_GRIPPER_MODE = 0
    
    uint32 GRIPPER_FORCE = 1
    
    uint32 GRIPPER_SPEED = 2
    
    uint32 GRIPPER_POSITION = 3
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GripperMode(null);
    return resolved;
    }
};

// Constants for message
GripperMode.Constants = {
  UNSPECIFIED_GRIPPER_MODE: 0,
  GRIPPER_FORCE: 1,
  GRIPPER_SPEED: 2,
  GRIPPER_POSITION: 3,
}

module.exports = GripperMode;
