// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BridgeConfig = require('../msg/BridgeConfig.js');

//-----------------------------------------------------------

let BridgeResult = require('../msg/BridgeResult.js');

//-----------------------------------------------------------

class EnableBridgeRequest {
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
        this.input = new BridgeConfig();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EnableBridgeRequest
    // Serialize message field [input]
    bufferOffset = BridgeConfig.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EnableBridgeRequest
    let len;
    let data = new EnableBridgeRequest(null);
    // Deserialize message field [input]
    data.input = BridgeConfig.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/EnableBridgeRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6562e0a872bdfbbb6cccfcddec4b1f8e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    BridgeConfig input
    
    ================================================================================
    MSG: kortex_driver/BridgeConfig
    
    uint32 device_identifier
    uint32 bridgetype
    BridgePortConfig port_config
    BridgeIdentifier bridge_id
    ================================================================================
    MSG: kortex_driver/BridgePortConfig
    
    uint32 target_port
    uint32 out_port
    ================================================================================
    MSG: kortex_driver/BridgeIdentifier
    
    uint32 bridge_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EnableBridgeRequest(null);
    if (msg.input !== undefined) {
      resolved.input = BridgeConfig.Resolve(msg.input)
    }
    else {
      resolved.input = new BridgeConfig()
    }

    return resolved;
    }
};

class EnableBridgeResponse {
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
        this.output = new BridgeResult();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EnableBridgeResponse
    // Serialize message field [output]
    bufferOffset = BridgeResult.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EnableBridgeResponse
    let len;
    let data = new EnableBridgeResponse(null);
    // Deserialize message field [output]
    data.output = BridgeResult.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/EnableBridgeResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '14fb5ca3149c75d17c29aa203a0186ee';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    BridgeResult output
    
    ================================================================================
    MSG: kortex_driver/BridgeResult
    
    BridgeIdentifier bridge_id
    uint32 status
    ================================================================================
    MSG: kortex_driver/BridgeIdentifier
    
    uint32 bridge_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EnableBridgeResponse(null);
    if (msg.output !== undefined) {
      resolved.output = BridgeResult.Resolve(msg.output)
    }
    else {
      resolved.output = new BridgeResult()
    }

    return resolved;
    }
};

module.exports = {
  Request: EnableBridgeRequest,
  Response: EnableBridgeResponse,
  md5sum() { return 'be9e21afaa7e92c6bf00b8eed13a695e'; },
  datatype() { return 'kortex_driver/EnableBridge'; }
};
