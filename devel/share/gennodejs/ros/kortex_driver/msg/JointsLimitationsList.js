// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let JointLimitation = require('./JointLimitation.js');

//-----------------------------------------------------------

class JointsLimitationsList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.joints_limitations = null;
    }
    else {
      if (initObj.hasOwnProperty('joints_limitations')) {
        this.joints_limitations = initObj.joints_limitations
      }
      else {
        this.joints_limitations = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type JointsLimitationsList
    // Serialize message field [joints_limitations]
    // Serialize the length for message field [joints_limitations]
    bufferOffset = _serializer.uint32(obj.joints_limitations.length, buffer, bufferOffset);
    obj.joints_limitations.forEach((val) => {
      bufferOffset = JointLimitation.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type JointsLimitationsList
    let len;
    let data = new JointsLimitationsList(null);
    // Deserialize message field [joints_limitations]
    // Deserialize array length for message field [joints_limitations]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.joints_limitations = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.joints_limitations[i] = JointLimitation.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 12 * object.joints_limitations.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/JointsLimitationsList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6dfe2c39e44634a2eba8f7879d518184';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    JointLimitation[] joints_limitations
    ================================================================================
    MSG: kortex_driver/JointLimitation
    
    uint32 joint_identifier
    uint32 type
    float32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new JointsLimitationsList(null);
    if (msg.joints_limitations !== undefined) {
      resolved.joints_limitations = new Array(msg.joints_limitations.length);
      for (let i = 0; i < resolved.joints_limitations.length; ++i) {
        resolved.joints_limitations[i] = JointLimitation.Resolve(msg.joints_limitations[i]);
      }
    }
    else {
      resolved.joints_limitations = []
    }

    return resolved;
    }
};

module.exports = JointsLimitationsList;
