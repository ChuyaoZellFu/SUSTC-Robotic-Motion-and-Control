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

class JointNavigationDirection {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointNavigationDirection
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointNavigationDirection
    let len;
    let data = new JointNavigationDirection(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/JointNavigationDirection';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '718d5eb2fe311f7d85c6686a0a7a3c9a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_JOINT_NAVIGATION_DIRECTION = 0
    
    uint32 JOINT_NEXT = 1
    
    uint32 JOINT_PREVIOUS = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointNavigationDirection(null);
    return resolved;
    }
};

// Constants for message
JointNavigationDirection.Constants = {
  UNSPECIFIED_JOINT_NAVIGATION_DIRECTION: 0,
  JOINT_NEXT: 1,
  JOINT_PREVIOUS: 2,
}

module.exports = JointNavigationDirection;
