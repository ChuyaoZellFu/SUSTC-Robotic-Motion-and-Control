// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let JointTorque = require('./JointTorque.js');

//-----------------------------------------------------------

class JointTorques {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joint_torques = null;
      this.duration = null;
    }
    else {
      if (initObj.hasOwnProperty('joint_torques')) {
        this.joint_torques = initObj.joint_torques
      }
      else {
        this.joint_torques = [];
      }
      if (initObj.hasOwnProperty('duration')) {
        this.duration = initObj.duration
      }
      else {
        this.duration = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointTorques
    // Serialize message field [joint_torques]
    // Serialize the length for message field [joint_torques]
    bufferOffset = _serializer.uint32(obj.joint_torques.length, buffer, bufferOffset);
    obj.joint_torques.forEach((val) => {
      bufferOffset = JointTorque.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [duration]
    bufferOffset = _serializer.uint32(obj.duration, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointTorques
    let len;
    let data = new JointTorques(null);
    // Deserialize message field [joint_torques]
    // Deserialize array length for message field [joint_torques]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.joint_torques = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.joint_torques[i] = JointTorque.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [duration]
    data.duration = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.joint_torques.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/JointTorques';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ada0e26af44ca932f53e6c0e450fadd8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    JointTorque[] joint_torques
    uint32 duration
    ================================================================================
    MSG: kortex_driver/JointTorque
    
    uint32 joint_identifier
    float32 value
    uint32 duration
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointTorques(null);
    if (msg.joint_torques !== undefined) {
      resolved.joint_torques = new Array(msg.joint_torques.length);
      for (let i = 0; i < resolved.joint_torques.length; ++i) {
        resolved.joint_torques[i] = JointTorque.Resolve(msg.joint_torques[i]);
      }
    }
    else {
      resolved.joint_torques = []
    }

    if (msg.duration !== undefined) {
      resolved.duration = msg.duration;
    }
    else {
      resolved.duration = 0
    }

    return resolved;
    }
};

module.exports = JointTorques;
