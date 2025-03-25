// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

let OperatingModeInformation = require('../msg/OperatingModeInformation.js');

//-----------------------------------------------------------

class GetOperatingModeRequest {
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
        this.input = new Empty();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetOperatingModeRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetOperatingModeRequest
    let len;
    let data = new GetOperatingModeRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetOperatingModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fa3403cd5897c9698bc0fdcb2a453fbc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Empty input
    
    ================================================================================
    MSG: kortex_driver/Empty
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetOperatingModeRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetOperatingModeResponse {
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
        this.output = new OperatingModeInformation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetOperatingModeResponse
    // Serialize message field [output]
    bufferOffset = OperatingModeInformation.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetOperatingModeResponse
    let len;
    let data = new GetOperatingModeResponse(null);
    // Deserialize message field [output]
    data.output = OperatingModeInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetOperatingModeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '323e2c52a1941a57eec708cf424bf367';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    OperatingModeInformation output
    
    ================================================================================
    MSG: kortex_driver/OperatingModeInformation
    
    uint32 operating_mode
    DeviceHandle device_handle
    ================================================================================
    MSG: kortex_driver/DeviceHandle
    
    uint32 device_type
    uint32 device_identifier
    uint32 order
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetOperatingModeResponse(null);
    if (msg.output !== undefined) {
      resolved.output = OperatingModeInformation.Resolve(msg.output)
    }
    else {
      resolved.output = new OperatingModeInformation()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetOperatingModeRequest,
  Response: GetOperatingModeResponse,
  md5sum() { return 'fe14b4b15e0f48ffd6cf705ff5a59c40'; },
  datatype() { return 'kortex_driver/GetOperatingMode'; }
};
