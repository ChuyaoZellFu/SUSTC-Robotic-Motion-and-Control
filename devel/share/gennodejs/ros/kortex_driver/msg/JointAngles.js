// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let JointAngle = require('./JointAngle.js');

//-----------------------------------------------------------

class JointAngles {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_angles = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_angles')) {
        this.joint_angles = initObj.joint_angles
      }
      else {
        this.joint_angles = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointAngles
    // Serialize message field [joint_angles]
    // Serialize the length for message field [joint_angles]
    bufferOffset = _serializer.uint32(obj.joint_angles.length, buffer, bufferOffset);
    obj.joint_angles.forEach((val) => {
      bufferOffset = JointAngle.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointAngles
    let len;
    let data = new JointAngles(null);
    // Deserialize message field [joint_angles]
    // Deserialize array length for message field [joint_angles]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.joint_angles = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.joint_angles[i] = JointAngle.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.joint_angles.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/JointAngles';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '94886d22261db2d621b5fe4c4bffdfa1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new JointAngles(null);
    if (msg.joint_angles !== undefined) {
      resolved.joint_angles = new Array(msg.joint_angles.length);
      for (let i = 0; i < resolved.joint_angles.length; ++i) {
        resolved.joint_angles[i] = JointAngle.Resolve(msg.joint_angles[i]);
      }
    }
    else {
      resolved.joint_angles = []
    }

    return resolved;
    }
};

module.exports = JointAngles;
