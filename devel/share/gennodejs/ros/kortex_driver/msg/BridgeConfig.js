// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BridgePortConfig = require('./BridgePortConfig.js');
let BridgeIdentifier = require('./BridgeIdentifier.js');

//-----------------------------------------------------------

class BridgeConfig {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.device_identifier = null;
      this.bridgetype = null;
      this.port_config = null;
      this.bridge_id = null;
    }
    else {
      if (initObj.hasOwnProperty('device_identifier')) {
        this.device_identifier = initObj.device_identifier
      }
      else {
        this.device_identifier = 0;
      }
      if (initObj.hasOwnProperty('bridgetype')) {
        this.bridgetype = initObj.bridgetype
      }
      else {
        this.bridgetype = 0;
      }
      if (initObj.hasOwnProperty('port_config')) {
        this.port_config = initObj.port_config
      }
      else {
        this.port_config = new BridgePortConfig();
      }
      if (initObj.hasOwnProperty('bridge_id')) {
        this.bridge_id = initObj.bridge_id
      }
      else {
        this.bridge_id = new BridgeIdentifier();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BridgeConfig
    // Serialize message field [device_identifier]
    bufferOffset = _serializer.uint32(obj.device_identifier, buffer, bufferOffset);
    // Serialize message field [bridgetype]
    bufferOffset = _serializer.uint32(obj.bridgetype, buffer, bufferOffset);
    // Serialize message field [port_config]
    bufferOffset = BridgePortConfig.serialize(obj.port_config, buffer, bufferOffset);
    // Serialize message field [bridge_id]
    bufferOffset = BridgeIdentifier.serialize(obj.bridge_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BridgeConfig
    let len;
    let data = new BridgeConfig(null);
    // Deserialize message field [device_identifier]
    data.device_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [bridgetype]
    data.bridgetype = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [port_config]
    data.port_config = BridgePortConfig.deserialize(buffer, bufferOffset);
    // Deserialize message field [bridge_id]
    data.bridge_id = BridgeIdentifier.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BridgeConfig';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '967c7a78caf96ff069310456674faf6a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new BridgeConfig(null);
    if (msg.device_identifier !== undefined) {
      resolved.device_identifier = msg.device_identifier;
    }
    else {
      resolved.device_identifier = 0
    }

    if (msg.bridgetype !== undefined) {
      resolved.bridgetype = msg.bridgetype;
    }
    else {
      resolved.bridgetype = 0
    }

    if (msg.port_config !== undefined) {
      resolved.port_config = BridgePortConfig.Resolve(msg.port_config)
    }
    else {
      resolved.port_config = new BridgePortConfig()
    }

    if (msg.bridge_id !== undefined) {
      resolved.bridge_id = BridgeIdentifier.Resolve(msg.bridge_id)
    }
    else {
      resolved.bridge_id = new BridgeIdentifier()
    }

    return resolved;
    }
};

module.exports = BridgeConfig;
