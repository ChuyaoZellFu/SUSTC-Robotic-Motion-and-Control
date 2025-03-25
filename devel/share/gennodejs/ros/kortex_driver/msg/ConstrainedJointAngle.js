// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');

//-----------------------------------------------------------

class ConstrainedJointAngle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_identifier = null;
      this.value = null;
      this.constraint = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_identifier')) {
        this.joint_identifier = initObj.joint_identifier
      }
      else {
        this.joint_identifier = 0;
      }
      if (initObj.hasOwnProperty('value')) {
        this.value = initObj.value
      }
      else {
        this.value = 0.0;
      }
      if (initObj.hasOwnProperty('constraint')) {
        this.constraint = initObj.constraint
      }
      else {
        this.constraint = new JointTrajectoryConstraint();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConstrainedJointAngle
    // Serialize message field [joint_identifier]
    bufferOffset = _serializer.uint32(obj.joint_identifier, buffer, bufferOffset);
    // Serialize message field [value]
    bufferOffset = _serializer.float32(obj.value, buffer, bufferOffset);
    // Serialize message field [constraint]
    bufferOffset = JointTrajectoryConstraint.serialize(obj.constraint, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConstrainedJointAngle
    let len;
    let data = new ConstrainedJointAngle(null);
    // Deserialize message field [joint_identifier]
    data.joint_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [value]
    data.value = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [constraint]
    data.constraint = JointTrajectoryConstraint.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ConstrainedJointAngle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6fb07b64148f47061948eb6c6ef38e00';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 joint_identifier
    float32 value
    JointTrajectoryConstraint constraint
    ================================================================================
    MSG: kortex_driver/JointTrajectoryConstraint
    
    uint32 type
    float32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConstrainedJointAngle(null);
    if (msg.joint_identifier !== undefined) {
      resolved.joint_identifier = msg.joint_identifier;
    }
    else {
      resolved.joint_identifier = 0
    }

    if (msg.value !== undefined) {
      resolved.value = msg.value;
    }
    else {
      resolved.value = 0.0
    }

    if (msg.constraint !== undefined) {
      resolved.constraint = JointTrajectoryConstraint.Resolve(msg.constraint)
    }
    else {
      resolved.constraint = new JointTrajectoryConstraint()
    }

    return resolved;
    }
};

module.exports = ConstrainedJointAngle;
