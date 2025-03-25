// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let BridgeIdentifier = require('./BridgeIdentifier.js');

//-----------------------------------------------------------

class BridgeResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bridge_id = null;
      this.status = null;
    }
    else {
      if (initObj.hasOwnProperty('bridge_id')) {
        this.bridge_id = initObj.bridge_id
      }
      else {
        this.bridge_id = new BridgeIdentifier();
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BridgeResult
    // Serialize message field [bridge_id]
    bufferOffset = BridgeIdentifier.serialize(obj.bridge_id, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.uint32(obj.status, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BridgeResult
    let len;
    let data = new BridgeResult(null);
    // Deserialize message field [bridge_id]
    data.bridge_id = BridgeIdentifier.deserialize(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BridgeResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c86e99efcef2f596ae80430e65714cbc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new BridgeResult(null);
    if (msg.bridge_id !== undefined) {
      resolved.bridge_id = BridgeIdentifier.Resolve(msg.bridge_id)
    }
    else {
      resolved.bridge_id = new BridgeIdentifier()
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = 0
    }

    return resolved;
    }
};

module.exports = BridgeResult;
