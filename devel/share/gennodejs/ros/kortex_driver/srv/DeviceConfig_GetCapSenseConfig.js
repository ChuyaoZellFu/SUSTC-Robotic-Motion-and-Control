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

let DeviceConfig_CapSenseConfig = require('../msg/DeviceConfig_CapSenseConfig.js');

//-----------------------------------------------------------

class DeviceConfig_GetCapSenseConfigRequest {
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
    // Serializes a message object of type DeviceConfig_GetCapSenseConfigRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DeviceConfig_GetCapSenseConfigRequest
    let len;
    let data = new DeviceConfig_GetCapSenseConfigRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/DeviceConfig_GetCapSenseConfigRequest';
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
    const resolved = new DeviceConfig_GetCapSenseConfigRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class DeviceConfig_GetCapSenseConfigResponse {
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
        this.output = new DeviceConfig_CapSenseConfig();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DeviceConfig_GetCapSenseConfigResponse
    // Serialize message field [output]
    bufferOffset = DeviceConfig_CapSenseConfig.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DeviceConfig_GetCapSenseConfigResponse
    let len;
    let data = new DeviceConfig_GetCapSenseConfigResponse(null);
    // Deserialize message field [output]
    data.output = DeviceConfig_CapSenseConfig.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/DeviceConfig_GetCapSenseConfigResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c228bd545e9c28a7c62fbd701f6cee7f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    DeviceConfig_CapSenseConfig output
    
    ================================================================================
    MSG: kortex_driver/DeviceConfig_CapSenseConfig
    
    uint32 mode
    float32 threshold_a
    float32 threshold_b
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DeviceConfig_GetCapSenseConfigResponse(null);
    if (msg.output !== undefined) {
      resolved.output = DeviceConfig_CapSenseConfig.Resolve(msg.output)
    }
    else {
      resolved.output = new DeviceConfig_CapSenseConfig()
    }

    return resolved;
    }
};

module.exports = {
  Request: DeviceConfig_GetCapSenseConfigRequest,
  Response: DeviceConfig_GetCapSenseConfigResponse,
  md5sum() { return '067bae44ab43b02bd4decf501211ef0c'; },
  datatype() { return 'kortex_driver/DeviceConfig_GetCapSenseConfig'; }
};
