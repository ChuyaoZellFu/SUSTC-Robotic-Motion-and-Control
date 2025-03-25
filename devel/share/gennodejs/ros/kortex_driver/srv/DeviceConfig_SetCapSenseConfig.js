// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let DeviceConfig_CapSenseConfig = require('../msg/DeviceConfig_CapSenseConfig.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class DeviceConfig_SetCapSenseConfigRequest {
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
        this.input = new DeviceConfig_CapSenseConfig();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DeviceConfig_SetCapSenseConfigRequest
    // Serialize message field [input]
    bufferOffset = DeviceConfig_CapSenseConfig.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DeviceConfig_SetCapSenseConfigRequest
    let len;
    let data = new DeviceConfig_SetCapSenseConfigRequest(null);
    // Deserialize message field [input]
    data.input = DeviceConfig_CapSenseConfig.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/DeviceConfig_SetCapSenseConfigRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dd8d7717ca7fec040fbe2437d8d5053b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    DeviceConfig_CapSenseConfig input
    
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
    const resolved = new DeviceConfig_SetCapSenseConfigRequest(null);
    if (msg.input !== undefined) {
      resolved.input = DeviceConfig_CapSenseConfig.Resolve(msg.input)
    }
    else {
      resolved.input = new DeviceConfig_CapSenseConfig()
    }

    return resolved;
    }
};

class DeviceConfig_SetCapSenseConfigResponse {
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
    // Serializes a message object of type DeviceConfig_SetCapSenseConfigResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DeviceConfig_SetCapSenseConfigResponse
    let len;
    let data = new DeviceConfig_SetCapSenseConfigResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/DeviceConfig_SetCapSenseConfigResponse';
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
    const resolved = new DeviceConfig_SetCapSenseConfigResponse(null);
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
  Request: DeviceConfig_SetCapSenseConfigRequest,
  Response: DeviceConfig_SetCapSenseConfigResponse,
  md5sum() { return 'af529d9737e7a99888f758db0a779b87'; },
  datatype() { return 'kortex_driver/DeviceConfig_SetCapSenseConfig'; }
};
