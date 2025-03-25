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

let WrenchLimitation = require('../msg/WrenchLimitation.js');

//-----------------------------------------------------------

class GetWrenchHardLimitationRequest {
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
    // Serializes a message object of type GetWrenchHardLimitationRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetWrenchHardLimitationRequest
    let len;
    let data = new GetWrenchHardLimitationRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetWrenchHardLimitationRequest';
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
    const resolved = new GetWrenchHardLimitationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetWrenchHardLimitationResponse {
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
        this.output = new WrenchLimitation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetWrenchHardLimitationResponse
    // Serialize message field [output]
    bufferOffset = WrenchLimitation.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetWrenchHardLimitationResponse
    let len;
    let data = new GetWrenchHardLimitationResponse(null);
    // Deserialize message field [output]
    data.output = WrenchLimitation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetWrenchHardLimitationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9cc5f6cd4c62deb4bcccd762d7e52acc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    WrenchLimitation output
    
    ================================================================================
    MSG: kortex_driver/WrenchLimitation
    
    float32 force
    float32 torque
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetWrenchHardLimitationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = WrenchLimitation.Resolve(msg.output)
    }
    else {
      resolved.output = new WrenchLimitation()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetWrenchHardLimitationRequest,
  Response: GetWrenchHardLimitationResponse,
  md5sum() { return 'd1a67d52edbd3c6e07b5922a22648cf3'; },
  datatype() { return 'kortex_driver/GetWrenchHardLimitation'; }
};
