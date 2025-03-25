// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Pose = require('./Pose.js');
let CartesianTrajectoryConstraint = require('./CartesianTrajectoryConstraint.js');

//-----------------------------------------------------------

class ConstrainedPose {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.target_pose = null;
      this.constraint = null;
    }
    else {
      if (initObj.hasOwnProperty('target_pose')) {
        this.target_pose = initObj.target_pose
      }
      else {
        this.target_pose = new Pose();
      }
      if (initObj.hasOwnProperty('constraint')) {
        this.constraint = initObj.constraint
      }
      else {
        this.constraint = new CartesianTrajectoryConstraint();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConstrainedPose
    // Serialize message field [target_pose]
    bufferOffset = Pose.serialize(obj.target_pose, buffer, bufferOffset);
    // Serialize message field [constraint]
    bufferOffset = CartesianTrajectoryConstraint.serialize(obj.constraint, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConstrainedPose
    let len;
    let data = new ConstrainedPose(null);
    // Deserialize message field [target_pose]
    data.target_pose = Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [constraint]
    data.constraint = CartesianTrajectoryConstraint.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += CartesianTrajectoryConstraint.getMessageSize(object.constraint);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ConstrainedPose';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '201b7431451c1b450ae81fca8778070f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    Pose target_pose
    CartesianTrajectoryConstraint constraint
    ================================================================================
    MSG: kortex_driver/Pose
    
    float32 x
    float32 y
    float32 z
    float32 theta_x
    float32 theta_y
    float32 theta_z
    ================================================================================
    MSG: kortex_driver/CartesianTrajectoryConstraint
    
    CartesianTrajectoryConstraint_type oneof_type
    ================================================================================
    MSG: kortex_driver/CartesianTrajectoryConstraint_type
    
    CartesianSpeed[] speed
    uint32[] duration
    ================================================================================
    MSG: kortex_driver/CartesianSpeed
    
    float32 translation
    float32 orientation
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConstrainedPose(null);
    if (msg.target_pose !== undefined) {
      resolved.target_pose = Pose.Resolve(msg.target_pose)
    }
    else {
      resolved.target_pose = new Pose()
    }

    if (msg.constraint !== undefined) {
      resolved.constraint = CartesianTrajectoryConstraint.Resolve(msg.constraint)
    }
    else {
      resolved.constraint = new CartesianTrajectoryConstraint()
    }

    return resolved;
    }
};

module.exports = ConstrainedPose;
