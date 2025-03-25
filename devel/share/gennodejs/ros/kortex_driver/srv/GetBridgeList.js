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

let BridgeList = require('../msg/BridgeList.js');

//-----------------------------------------------------------

class GetBridgeListRequest {
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
    // Serializes a message object of type GetBridgeListRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetBridgeListRequest
    let len;
    let data = new GetBridgeListRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetBridgeListRequest';
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
    const resolved = new GetBridgeListRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetBridgeListResponse {
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
        this.output = new BridgeList();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetBridgeListResponse
    // Serialize message field [output]
    bufferOffset = BridgeList.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetBridgeListResponse
    let len;
    let data = new GetBridgeListResponse(null);
    // Deserialize message field [output]
    data.output = BridgeList.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += BridgeList.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetBridgeListResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bf36ac202e627c34848b7cda25e6a013';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    BridgeList output
    
    ================================================================================
    MSG: kortex_driver/BridgeList
    
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
    const resolved = new GetBridgeListResponse(null);
    if (msg.output !== undefined) {
      resolved.output = BridgeList.Resolve(msg.output)
    }
    else {
      resolved.output = new BridgeList()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetBridgeListRequest,
  Response: GetBridgeListResponse,
  md5sum() { return '866f5d10e4d60286569bdfe7c65c5078'; },
  datatype() { return 'kortex_driver/GetBridgeList'; }
};
