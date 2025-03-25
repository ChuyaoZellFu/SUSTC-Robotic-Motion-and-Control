// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let GPIOIdentification = require('../msg/GPIOIdentification.js');

//-----------------------------------------------------------

let GPIOState = require('../msg/GPIOState.js');

//-----------------------------------------------------------

class GetGPIOStateRequest {
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
        this.input = new GPIOIdentification();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetGPIOStateRequest
    // Serialize message field [input]
    bufferOffset = GPIOIdentification.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetGPIOStateRequest
    let len;
    let data = new GetGPIOStateRequest(null);
    // Deserialize message field [input]
    data.input = GPIOIdentification.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetGPIOStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '28fc5544fa28c41b8590a221a4f4db30';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    GPIOIdentification input
    
    ================================================================================
    MSG: kortex_driver/GPIOIdentification
    
    uint32 identifier
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetGPIOStateRequest(null);
    if (msg.input !== undefined) {
      resolved.input = GPIOIdentification.Resolve(msg.input)
    }
    else {
      resolved.input = new GPIOIdentification()
    }

    return resolved;
    }
};

class GetGPIOStateResponse {
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
        this.output = new GPIOState();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetGPIOStateResponse
    // Serialize message field [output]
    bufferOffset = GPIOState.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetGPIOStateResponse
    let len;
    let data = new GetGPIOStateResponse(null);
    // Deserialize message field [output]
    data.output = GPIOState.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetGPIOStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b981cf2cc036ee10c4e764efbf7efdd9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    GPIOState output
    
    ================================================================================
    MSG: kortex_driver/GPIOState
    
    uint32 identifier
    uint32 value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetGPIOStateResponse(null);
    if (msg.output !== undefined) {
      resolved.output = GPIOState.Resolve(msg.output)
    }
    else {
      resolved.output = new GPIOState()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetGPIOStateRequest,
  Response: GetGPIOStateResponse,
  md5sum() { return 'ee1aa8c7cecf549eed7751c07a3eb88c'; },
  datatype() { return 'kortex_driver/GetGPIOState'; }
};
