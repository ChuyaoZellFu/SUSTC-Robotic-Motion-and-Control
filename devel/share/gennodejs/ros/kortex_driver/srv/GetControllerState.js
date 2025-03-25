// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControllerHandle = require('../msg/ControllerHandle.js');

//-----------------------------------------------------------

let ControllerState = require('../msg/ControllerState.js');

//-----------------------------------------------------------

class GetControllerStateRequest {
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
        this.input = new ControllerHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetControllerStateRequest
    // Serialize message field [input]
    bufferOffset = ControllerHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetControllerStateRequest
    let len;
    let data = new GetControllerStateRequest(null);
    // Deserialize message field [input]
    data.input = ControllerHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetControllerStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0d6670247f140ab183942a0576f87f5f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ControllerHandle input
    
    ================================================================================
    MSG: kortex_driver/ControllerHandle
    
    uint32 type
    uint32 controller_identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetControllerStateRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ControllerHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new ControllerHandle()
    }

    return resolved;
    }
};

class GetControllerStateResponse {
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
        this.output = new ControllerState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetControllerStateResponse
    // Serialize message field [output]
    bufferOffset = ControllerState.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetControllerStateResponse
    let len;
    let data = new GetControllerStateResponse(null);
    // Deserialize message field [output]
    data.output = ControllerState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetControllerStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '91a45b957c71af1f17aba3256ed6d6b5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ControllerState output
    
    ================================================================================
    MSG: kortex_driver/ControllerState
    
    ControllerHandle handle
    uint32 event_type
    ================================================================================
    MSG: kortex_driver/ControllerHandle
    
    uint32 type
    uint32 controller_identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetControllerStateResponse(null);
    if (msg.output !== undefined) {
      resolved.output = ControllerState.Resolve(msg.output)
    }
    else {
      resolved.output = new ControllerState()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetControllerStateRequest,
  Response: GetControllerStateResponse,
  md5sum() { return 'c3a3b583b8a6198c6f61bb7c1bde10f0'; },
  datatype() { return 'kortex_driver/GetControllerState'; }
};
