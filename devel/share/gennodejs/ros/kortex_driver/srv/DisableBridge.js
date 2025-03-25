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

let BridgeResult = require('../msg/BridgeResult.js');

//-----------------------------------------------------------

class DisableBridgeRequest {
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
    // Serializes a message object of type DisableBridgeRequest
    // Serialize message field [input]
    bufferOffset = BridgeIdentifier.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DisableBridgeRequest
    let len;
    let data = new DisableBridgeRequest(null);
    // Deserialize message field [input]
    data.input = BridgeIdentifier.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/DisableBridgeRequest';
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
    const resolved = new DisableBridgeRequest(null);
    if (msg.input !== undefined) {
      resolved.input = BridgeIdentifier.Resolve(msg.input)
    }
    else {
      resolved.input = new BridgeIdentifier()
    }

    return resolved;
    }
};

class DisableBridgeResponse {
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
    // Serializes a message object of type DisableBridgeResponse
    // Serialize message field [output]
    bufferOffset = BridgeResult.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DisableBridgeResponse
    let len;
    let data = new DisableBridgeResponse(null);
    // Deserialize message field [output]
    data.output = BridgeResult.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/DisableBridgeResponse';
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
    const resolved = new DisableBridgeResponse(null);
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
  Request: DisableBridgeRequest,
  Response: DisableBridgeResponse,
  md5sum() { return '5d03590e711a693b2b6ee914ec9cc8b3'; },
  datatype() { return 'kortex_driver/DisableBridge'; }
};
