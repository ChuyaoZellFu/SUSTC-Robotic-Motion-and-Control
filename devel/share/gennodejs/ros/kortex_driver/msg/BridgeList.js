// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BridgeConfig = require('./BridgeConfig.js');

//-----------------------------------------------------------

class BridgeList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bridgeConfig = null;
    }
    else {
      if (initObj.hasOwnProperty('bridgeConfig')) {
        this.bridgeConfig = initObj.bridgeConfig
      }
      else {
        this.bridgeConfig = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BridgeList
    // Serialize message field [bridgeConfig]
    // Serialize the length for message field [bridgeConfig]
    bufferOffset = _serializer.uint32(obj.bridgeConfig.length, buffer, bufferOffset);
    obj.bridgeConfig.forEach((val) => {
      bufferOffset = BridgeConfig.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BridgeList
    let len;
    let data = new BridgeList(null);
    // Deserialize message field [bridgeConfig]
    // Deserialize array length for message field [bridgeConfig]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.bridgeConfig = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.bridgeConfig[i] = BridgeConfig.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 20 * object.bridgeConfig.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BridgeList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '67edf56b8cf474f747335e24413448f0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    BridgeConfig[] bridgeConfig
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
    const resolved = new BridgeList(null);
    if (msg.bridgeConfig !== undefined) {
      resolved.bridgeConfig = new Array(msg.bridgeConfig.length);
      for (let i = 0; i < resolved.bridgeConfig.length; ++i) {
        resolved.bridgeConfig[i] = BridgeConfig.Resolve(msg.bridgeConfig[i]);
      }
    }
    else {
      resolved.bridgeConfig = []
    }

    return resolved;
    }
};

module.exports = BridgeList;
