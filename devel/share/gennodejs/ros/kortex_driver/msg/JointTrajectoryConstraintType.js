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

class JointTrajectoryConstraintType {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointTrajectoryConstraintType
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointTrajectoryConstraintType
    let len;
    let data = new JointTrajectoryConstraintType(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/JointTrajectoryConstraintType';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a007b68cae061ff813f21ddba74d4035';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_JOINT_CONSTRAINT = 0
    
    uint32 JOINT_CONSTRAINT_DURATION = 1
    
    uint32 JOINT_CONSTRAINT_SPEED = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointTrajectoryConstraintType(null);
    return resolved;
    }
};

// Constants for message
JointTrajectoryConstraintType.Constants = {
  UNSPECIFIED_JOINT_CONSTRAINT: 0,
  JOINT_CONSTRAINT_DURATION: 1,
  JOINT_CONSTRAINT_SPEED: 2,
}

module.exports = JointTrajectoryConstraintType;
