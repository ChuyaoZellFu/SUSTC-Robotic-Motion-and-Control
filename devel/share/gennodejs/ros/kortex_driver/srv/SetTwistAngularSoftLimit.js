// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let TwistAngularSoftLimit = require('../msg/TwistAngularSoftLimit.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetTwistAngularSoftLimitRequest {
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
        this.input = new TwistAngularSoftLimit();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetTwistAngularSoftLimitRequest
    // Serialize message field [input]
    bufferOffset = TwistAngularSoftLimit.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetTwistAngularSoftLimitRequest
    let len;
    let data = new SetTwistAngularSoftLimitRequest(null);
    // Deserialize message field [input]
    data.input = TwistAngularSoftLimit.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetTwistAngularSoftLimitRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a60ca87a9438a0869373a4e14ef5f807';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    TwistAngularSoftLimit input
    
    ================================================================================
    MSG: kortex_driver/TwistAngularSoftLimit
    
    uint32 control_mode
    float32 twist_angular_soft_limit
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetTwistAngularSoftLimitRequest(null);
    if (msg.input !== undefined) {
      resolved.input = TwistAngularSoftLimit.Resolve(msg.input)
    }
    else {
      resolved.input = new TwistAngularSoftLimit()
    }

    return resolved;
    }
};

class SetTwistAngularSoftLimitResponse {
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
    // Serializes a message object of type SetTwistAngularSoftLimitResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetTwistAngularSoftLimitResponse
    let len;
    let data = new SetTwistAngularSoftLimitResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetTwistAngularSoftLimitResponse';
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
    const resolved = new SetTwistAngularSoftLimitResponse(null);
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
  Request: SetTwistAngularSoftLimitRequest,
  Response: SetTwistAngularSoftLimitResponse,
  md5sum() { return '3b523deda9069339b963e073a605096e'; },
  datatype() { return 'kortex_driver/SetTwistAngularSoftLimit'; }
};
