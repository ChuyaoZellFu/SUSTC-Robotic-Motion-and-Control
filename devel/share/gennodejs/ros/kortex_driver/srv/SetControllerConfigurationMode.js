// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControllerConfigurationMode = require('../msg/ControllerConfigurationMode.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetControllerConfigurationModeRequest {
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
        this.input = new ControllerConfigurationMode();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetControllerConfigurationModeRequest
    // Serialize message field [input]
    bufferOffset = ControllerConfigurationMode.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetControllerConfigurationModeRequest
    let len;
    let data = new SetControllerConfigurationModeRequest(null);
    // Deserialize message field [input]
    data.input = ControllerConfigurationMode.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetControllerConfigurationModeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8a1806dd909a81cda18bb43787efba54';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ControllerConfigurationMode input
    
    ================================================================================
    MSG: kortex_driver/ControllerConfigurationMode
    
    bool enable
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetControllerConfigurationModeRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ControllerConfigurationMode.Resolve(msg.input)
    }
    else {
      resolved.input = new ControllerConfigurationMode()
    }

    return resolved;
    }
};

class SetControllerConfigurationModeResponse {
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
    // Serializes a message object of type SetControllerConfigurationModeResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetControllerConfigurationModeResponse
    let len;
    let data = new SetControllerConfigurationModeResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetControllerConfigurationModeResponse';
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
    const resolved = new SetControllerConfigurationModeResponse(null);
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
  Request: SetControllerConfigurationModeRequest,
  Response: SetControllerConfigurationModeResponse,
  md5sum() { return 'a1e10ce64919ae3ba0f5b2193b693ecb'; },
  datatype() { return 'kortex_driver/SetControllerConfigurationMode'; }
};
