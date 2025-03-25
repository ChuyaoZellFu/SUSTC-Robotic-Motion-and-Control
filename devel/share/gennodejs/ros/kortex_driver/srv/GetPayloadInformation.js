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

let PayloadInformation = require('../msg/PayloadInformation.js');

//-----------------------------------------------------------

class GetPayloadInformationRequest {
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
    // Serializes a message object of type GetPayloadInformationRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPayloadInformationRequest
    let len;
    let data = new GetPayloadInformationRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetPayloadInformationRequest';
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
    const resolved = new GetPayloadInformationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetPayloadInformationResponse {
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
        this.output = new PayloadInformation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPayloadInformationResponse
    // Serialize message field [output]
    bufferOffset = PayloadInformation.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPayloadInformationResponse
    let len;
    let data = new GetPayloadInformationResponse(null);
    // Deserialize message field [output]
    data.output = PayloadInformation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetPayloadInformationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ee7b6d48bea0278cf84502742813a923';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    PayloadInformation output
    
    ================================================================================
    MSG: kortex_driver/PayloadInformation
    
    float32 payload_mass
    ControlConfig_Position payload_mass_center
    ================================================================================
    MSG: kortex_driver/ControlConfig_Position
    
    float32 x
    float32 y
    float32 z
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPayloadInformationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = PayloadInformation.Resolve(msg.output)
    }
    else {
      resolved.output = new PayloadInformation()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetPayloadInformationRequest,
  Response: GetPayloadInformationResponse,
  md5sum() { return '38a744b19ddbb71fb4d7e8724de570f6'; },
  datatype() { return 'kortex_driver/GetPayloadInformation'; }
};
