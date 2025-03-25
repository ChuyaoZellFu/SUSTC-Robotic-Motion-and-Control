// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let SafetyHandle = require('../msg/SafetyHandle.js');

//-----------------------------------------------------------

let SafetyStatus = require('../msg/SafetyStatus.js');

//-----------------------------------------------------------

class GetSafetyStatusRequest {
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
        this.input = new SafetyHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSafetyStatusRequest
    // Serialize message field [input]
    bufferOffset = SafetyHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSafetyStatusRequest
    let len;
    let data = new GetSafetyStatusRequest(null);
    // Deserialize message field [input]
    data.input = SafetyHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetSafetyStatusRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '28fc5544fa28c41b8590a221a4f4db30';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SafetyHandle input
    
    ================================================================================
    MSG: kortex_driver/SafetyHandle
    
    uint32 identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSafetyStatusRequest(null);
    if (msg.input !== undefined) {
      resolved.input = SafetyHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new SafetyHandle()
    }

    return resolved;
    }
};

class GetSafetyStatusResponse {
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
        this.output = new SafetyStatus();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetSafetyStatusResponse
    // Serialize message field [output]
    bufferOffset = SafetyStatus.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetSafetyStatusResponse
    let len;
    let data = new GetSafetyStatusResponse(null);
    // Deserialize message field [output]
    data.output = SafetyStatus.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetSafetyStatusResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a325384bd39bef34e5243a4f8547a90d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    SafetyStatus output
    
    ================================================================================
    MSG: kortex_driver/SafetyStatus
    
    uint32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetSafetyStatusResponse(null);
    if (msg.output !== undefined) {
      resolved.output = SafetyStatus.Resolve(msg.output)
    }
    else {
      resolved.output = new SafetyStatus()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetSafetyStatusRequest,
  Response: GetSafetyStatusResponse,
  md5sum() { return '0b50aa56e0379c3a0c319bd4962efe2e'; },
  datatype() { return 'kortex_driver/GetSafetyStatus'; }
};
