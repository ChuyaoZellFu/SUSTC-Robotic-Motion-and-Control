// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControlConfig_JointSpeeds = require('../msg/ControlConfig_JointSpeeds.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetDesiredJointSpeedsRequest {
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
        this.input = new ControlConfig_JointSpeeds();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetDesiredJointSpeedsRequest
    // Serialize message field [input]
    bufferOffset = ControlConfig_JointSpeeds.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetDesiredJointSpeedsRequest
    let len;
    let data = new SetDesiredJointSpeedsRequest(null);
    // Deserialize message field [input]
    data.input = ControlConfig_JointSpeeds.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ControlConfig_JointSpeeds.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetDesiredJointSpeedsRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '818ed98df91d7c2f80e41de1e13f713c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ControlConfig_JointSpeeds input
    
    ================================================================================
    MSG: kortex_driver/ControlConfig_JointSpeeds
    
    float32[] joint_speed
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetDesiredJointSpeedsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ControlConfig_JointSpeeds.Resolve(msg.input)
    }
    else {
      resolved.input = new ControlConfig_JointSpeeds()
    }

    return resolved;
    }
};

class SetDesiredJointSpeedsResponse {
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
    // Serializes a message object of type SetDesiredJointSpeedsResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetDesiredJointSpeedsResponse
    let len;
    let data = new SetDesiredJointSpeedsResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetDesiredJointSpeedsResponse';
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
    const resolved = new SetDesiredJointSpeedsResponse(null);
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
  Request: SetDesiredJointSpeedsRequest,
  Response: SetDesiredJointSpeedsResponse,
  md5sum() { return '57cd70ec5519968c6e959068a3575705'; },
  datatype() { return 'kortex_driver/SetDesiredJointSpeeds'; }
};
