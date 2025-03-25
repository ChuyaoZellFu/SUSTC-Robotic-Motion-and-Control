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
let JointAngles = require('./JointAngles.js');

//-----------------------------------------------------------

class IKData {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cartesian_pose = null;
      this.guess = null;
    }
    else {
      if (initObj.hasOwnProperty('cartesian_pose')) {
        this.cartesian_pose = initObj.cartesian_pose
      }
      else {
        this.cartesian_pose = new Pose();
      }
      if (initObj.hasOwnProperty('guess')) {
        this.guess = initObj.guess
      }
      else {
        this.guess = new JointAngles();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type IKData
    // Serialize message field [cartesian_pose]
    bufferOffset = Pose.serialize(obj.cartesian_pose, buffer, bufferOffset);
    // Serialize message field [guess]
    bufferOffset = JointAngles.serialize(obj.guess, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type IKData
    let len;
    let data = new IKData(null);
    // Deserialize message field [cartesian_pose]
    data.cartesian_pose = Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [guess]
    data.guess = JointAngles.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += JointAngles.getMessageSize(object.guess);
    return length + 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/IKData';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '29f05c9210572828af7df145fee29d3b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    Pose cartesian_pose
    JointAngles guess
    ================================================================================
    MSG: kortex_driver/Pose
    
    float32 x
    float32 y
    float32 z
    float32 theta_x
    float32 theta_y
    float32 theta_z
    ================================================================================
    MSG: kortex_driver/JointAngles
    
    JointAngle[] joint_angles
    ================================================================================
    MSG: kortex_driver/JointAngle
    
    uint32 joint_identifier
    float32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new IKData(null);
    if (msg.cartesian_pose !== undefined) {
      resolved.cartesian_pose = Pose.Resolve(msg.cartesian_pose)
    }
    else {
      resolved.cartesian_pose = new Pose()
    }

    if (msg.guess !== undefined) {
      resolved.guess = JointAngles.Resolve(msg.guess)
    }
    else {
      resolved.guess = new JointAngles()
    }

    return resolved;
    }
};

module.exports = IKData;
