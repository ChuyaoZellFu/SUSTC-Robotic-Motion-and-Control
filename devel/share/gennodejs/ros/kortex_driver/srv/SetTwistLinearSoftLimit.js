// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TwistLinearSoftLimit = require('../msg/TwistLinearSoftLimit.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetTwistLinearSoftLimitRequest {
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
        this.input = new TwistLinearSoftLimit();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetTwistLinearSoftLimitRequest
    // Serialize message field [input]
    bufferOffset = TwistLinearSoftLimit.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetTwistLinearSoftLimitRequest
    let len;
    let data = new SetTwistLinearSoftLimitRequest(null);
    // Deserialize message field [input]
    data.input = TwistLinearSoftLimit.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetTwistLinearSoftLimitRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '95de4514f4e949929bc28d380ab29978';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    TwistLinearSoftLimit input
    
    ================================================================================
    MSG: kortex_driver/TwistLinearSoftLimit
    
    uint32 control_mode
    float32 twist_linear_soft_limit
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetTwistLinearSoftLimitRequest(null);
    if (msg.input !== undefined) {
      resolved.input = TwistLinearSoftLimit.Resolve(msg.input)
    }
    else {
      resolved.input = new TwistLinearSoftLimit()
    }

    return resolved;
    }
};

class SetTwistLinearSoftLimitResponse {
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
    // Serializes a message object of type SetTwistLinearSoftLimitResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetTwistLinearSoftLimitResponse
    let len;
    let data = new SetTwistLinearSoftLimitResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetTwistLinearSoftLimitResponse';
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
    const resolved = new SetTwistLinearSoftLimitResponse(null);
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
  Request: SetTwistLinearSoftLimitRequest,
  Response: SetTwistLinearSoftLimitResponse,
  md5sum() { return '1641ba0fe5229f88d9cda0ee47099b0f'; },
  datatype() { return 'kortex_driver/SetTwistLinearSoftLimit'; }
};
