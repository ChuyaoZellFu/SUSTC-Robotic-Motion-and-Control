// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class BridgeIdentifier {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.bridge_id = null;
    }
    else {
      if (initObj.hasOwnProperty('bridge_id')) {
        this.bridge_id = initObj.bridge_id
      }
      else {
        this.bridge_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BridgeIdentifier
    // Serialize message field [bridge_id]
    bufferOffset = _serializer.uint32(obj.bridge_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BridgeIdentifier
    let len;
    let data = new BridgeIdentifier(null);
    // Deserialize message field [bridge_id]
    data.bridge_id = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/BridgeIdentifier';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '24fa834626ec2b7a0c103b00131d6015';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 bridge_id
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BridgeIdentifier(null);
    if (msg.bridge_id !== undefined) {
      resolved.bridge_id = msg.bridge_id;
    }
    else {
      resolved.bridge_id = 0
    }

    return resolved;
    }
};

module.exports = BridgeIdentifier;
