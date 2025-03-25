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

let ControlConfig_ControlModeInformation = require('../msg/ControlConfig_ControlModeInformation.js');

//-----------------------------------------------------------

class ControlConfig_GetControlModeRequest {
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
    // Serializes a message object of type ControlConfig_GetControlModeRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlConfig_GetControlModeRequest
    let len;
    let data = new ControlConfig_GetControlModeRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ControlConfig_GetControlModeRequest';
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
    const resolved = new ControlConfig_GetControlModeRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class ControlConfig_GetControlModeResponse {
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
        this.output = new ControlConfig_ControlModeInformation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlConfig_GetControlModeResponse
    // Serialize message field [output]
    bufferOffset = ControlConfig_ControlModeInformation.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlConfig_GetControlModeResponse
    let len;
    let data = new ControlConfig_GetControlModeResponse(null);
    // Deserialize message field [output]
    data.output = ControlConfig_ControlModeInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ControlConfig_GetControlModeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e3d508e1e33b4af9f591f4bf29eb71d5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ControlConfig_ControlModeInformation output
    
    ================================================================================
    MSG: kortex_driver/ControlConfig_ControlModeInformation
    
    uint32 control_mode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlConfig_GetControlModeResponse(null);
    if (msg.output !== undefined) {
      resolved.output = ControlConfig_ControlModeInformation.Resolve(msg.output)
    }
    else {
      resolved.output = new ControlConfig_ControlModeInformation()
    }

    return resolved;
    }
};

module.exports = {
  Request: ControlConfig_GetControlModeRequest,
  Response: ControlConfig_GetControlModeResponse,
  md5sum() { return 'da0700ae55dc292dd80209569315d03f'; },
  datatype() { return 'kortex_driver/ControlConfig_GetControlMode'; }
};
