// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let OperatingModeInformation = require('../msg/OperatingModeInformation.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetOperatingModeRequest {
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
        this.input = new OperatingModeInformation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetOperatingModeRequest
    // Serialize message field [input]
    bufferOffset = OperatingModeInformation.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetOperatingModeRequest
    let len;
    let data = new SetOperatingModeRequest(null);
    // Deserialize message field [input]
    data.input = OperatingModeInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetOperatingModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '10045b5e4422500c8ee87d01a4bbdac6';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    OperatingModeInformation input
    
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
    const resolved = new SetOperatingModeRequest(null);
    if (msg.input !== undefined) {
      resolved.input = OperatingModeInformation.Resolve(msg.input)
    }
    else {
      resolved.input = new OperatingModeInformation()
    }

    return resolved;
    }
};

class SetOperatingModeResponse {
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
    // Serializes a message object of type SetOperatingModeResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetOperatingModeResponse
    let len;
    let data = new SetOperatingModeResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetOperatingModeResponse';
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
    const resolved = new SetOperatingModeResponse(null);
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
  Request: SetOperatingModeRequest,
  Response: SetOperatingModeResponse,
  md5sum() { return 'c4131bf69b8a60653e5e053fa2ce9151'; },
  datatype() { return 'kortex_driver/SetOperatingMode'; }
};
