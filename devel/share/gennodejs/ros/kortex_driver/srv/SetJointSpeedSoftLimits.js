// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let JointSpeedSoftLimits = require('../msg/JointSpeedSoftLimits.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetJointSpeedSoftLimitsRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.input = null;
    }
    else {
      if (initObj.hasOwnProperty('input')) {
        this.input = initObj.input
      }
      else {
        this.input = new JointSpeedSoftLimits();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetJointSpeedSoftLimitsRequest
    // Serialize message field [input]
    bufferOffset = JointSpeedSoftLimits.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetJointSpeedSoftLimitsRequest
    let len;
    let data = new SetJointSpeedSoftLimitsRequest(null);
    // Deserialize message field [input]
    data.input = JointSpeedSoftLimits.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += JointSpeedSoftLimits.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetJointSpeedSoftLimitsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '830ffce6732ff2d0ba66feefa07cfd66';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    JointSpeedSoftLimits input
    
    ================================================================================
    MSG: kortex_driver/JointSpeedSoftLimits
    
    uint32 control_mode
    float32[] joint_speed_soft_limits
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetJointSpeedSoftLimitsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = JointSpeedSoftLimits.Resolve(msg.input)
    }
    else {
      resolved.input = new JointSpeedSoftLimits()
    }

    return resolved;
    }
};

class SetJointSpeedSoftLimitsResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.output = null;
    }
    else {
      if (initObj.hasOwnProperty('output')) {
        this.output = initObj.output
      }
      else {
        this.output = new Empty();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetJointSpeedSoftLimitsResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetJointSpeedSoftLimitsResponse
    let len;
    let data = new SetJointSpeedSoftLimitsResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetJointSpeedSoftLimitsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c6c43d221c810050f75091660f63b0cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Empty output
    
    ================================================================================
    MSG: kortex_driver/Empty
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetJointSpeedSoftLimitsResponse(null);
    if (msg.output !== undefined) {
      resolved.output = Empty.Resolve(msg.output)
    }
    else {
      resolved.output = new Empty()
    }

    return resolved;
    }
};

module.exports = {
  Request: SetJointSpeedSoftLimitsRequest,
  Response: SetJointSpeedSoftLimitsResponse,
  md5sum() { return 'f67059c3b3849506a74d17610ccdb240'; },
  datatype() { return 'kortex_driver/SetJointSpeedSoftLimits'; }
};
