// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let KinematicLimits = require('./KinematicLimits.js');

//-----------------------------------------------------------

class KinematicLimitsList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.kinematic_limits_list = null;
    }
    else {
      if (initObj.hasOwnProperty('kinematic_limits_list')) {
        this.kinematic_limits_list = initObj.kinematic_limits_list
      }
      else {
        this.kinematic_limits_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type KinematicLimitsList
    // Serialize message field [kinematic_limits_list]
    // Serialize the length for message field [kinematic_limits_list]
    bufferOffset = _serializer.uint32(obj.kinematic_limits_list.length, buffer, bufferOffset);
    obj.kinematic_limits_list.forEach((val) => {
      bufferOffset = KinematicLimits.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type KinematicLimitsList
    let len;
    let data = new KinematicLimitsList(null);
    // Deserialize message field [kinematic_limits_list]
    // Deserialize array length for message field [kinematic_limits_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.kinematic_limits_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.kinematic_limits_list[i] = KinematicLimits.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.kinematic_limits_list.forEach((val) => {
      length += KinematicLimits.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/KinematicLimitsList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '85620e63d1ea4e2fbf21a5cb16cb2964';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    KinematicLimits[] kinematic_limits_list
    ================================================================================
    MSG: kortex_driver/KinematicLimits
    
    uint32 control_mode
    float32 twist_linear
    float32 twist_angular
    float32[] joint_speed_limits
    float32[] joint_acceleration_limits
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new KinematicLimitsList(null);
    if (msg.kinematic_limits_list !== undefined) {
      resolved.kinematic_limits_list = new Array(msg.kinematic_limits_list.length);
      for (let i = 0; i < resolved.kinematic_limits_list.length; ++i) {
        resolved.kinematic_limits_list[i] = KinematicLimits.Resolve(msg.kinematic_limits_list[i]);
      }
    }
    else {
      resolved.kinematic_limits_list = []
    }

    return resolved;
    }
};

module.exports = KinematicLimitsList;
