// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BridgeIdentifier = require('../msg/BridgeIdentifier.js');

//-----------------------------------------------------------

let BridgeConfig = require('../msg/BridgeConfig.js');

//-----------------------------------------------------------

class GetBridgeConfigRequest {
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
        this.input = new BridgeIdentifier();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetBridgeConfigRequest
    // Serialize message field [input]
    bufferOffset = BridgeIdentifier.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetBridgeConfigRequest
    let len;
    let data = new GetBridgeConfigRequest(null);
    // Deserialize message field [input]
    data.input = BridgeIdentifier.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetBridgeConfigRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e7fff85c040dee9f9ad9fd35ed6100dc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    BridgeIdentifier input
    
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
    const resolved = new GetBridgeConfigRequest(null);
    if (msg.input !== undefined) {
      resolved.input = BridgeIdentifier.Resolve(msg.input)
    }
    else {
      resolved.input = new BridgeIdentifier()
    }

    return resolved;
    }
};

class GetBridgeConfigResponse {
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
        this.output = new BridgeConfig();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetBridgeConfigResponse
    // Serialize message field [output]
    bufferOffset = BridgeConfig.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetBridgeConfigResponse
    let len;
    let data = new GetBridgeConfigResponse(null);
    // Deserialize message field [output]
    data.output = BridgeConfig.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetBridgeConfigResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c577cd4a3a5ea7f0303fdae8be3d2ad4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    BridgeConfig output
    
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
    const resolved = new GetBridgeConfigResponse(null);
    if (msg.output !== undefined) {
      resolved.output = BridgeConfig.Resolve(msg.output)
    }
    else {
      resolved.output = new BridgeConfig()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetBridgeConfigRequest,
  Response: GetBridgeConfigResponse,
  md5sum() { return '9f7aa46e9ca64f4d1fa2fe2894e0afa6'; },
  datatype() { return 'kortex_driver/GetBridgeConfig'; }
};
