// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let JointAngles = require('./JointAngles.js');
let JointTrajectoryConstraint = require('./JointTrajectoryConstraint.js');

//-----------------------------------------------------------

class ConstrainedJointAngles {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_angles = null;
      this.constraint = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_angles')) {
        this.joint_angles = initObj.joint_angles
      }
      else {
        this.joint_angles = new JointAngles();
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
    // Serializes a message object of type ConstrainedJointAngles
    // Serialize message field [joint_angles]
    bufferOffset = JointAngles.serialize(obj.joint_angles, buffer, bufferOffset);
    // Serialize message field [constraint]
    bufferOffset = JointTrajectoryConstraint.serialize(obj.constraint, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConstrainedJointAngles
    let len;
    let data = new ConstrainedJointAngles(null);
    // Deserialize message field [joint_angles]
    data.joint_angles = JointAngles.deserialize(buffer, bufferOffset);
    // Deserialize message field [constraint]
    data.constraint = JointTrajectoryConstraint.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += JointAngles.getMessageSize(object.joint_angles);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ConstrainedJointAngles';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '75faad16493b9d4c290ef3b7fb5a8947';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    JointAngles joint_angles
    JointTrajectoryConstraint constraint
    ================================================================================
    MSG: kortex_driver/JointAngles
    
    JointAngle[] joint_angles
    ================================================================================
    MSG: kortex_driver/JointAngle
    
    uint32 joint_identifier
    float32 value
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
    const resolved = new ConstrainedJointAngles(null);
    if (msg.joint_angles !== undefined) {
      resolved.joint_angles = JointAngles.Resolve(msg.joint_angles)
    }
    else {
      resolved.joint_angles = new JointAngles()
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

module.exports = ConstrainedJointAngles;
