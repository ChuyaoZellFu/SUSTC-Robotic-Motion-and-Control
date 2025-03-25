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

let TwistLimitation = require('../msg/TwistLimitation.js');

//-----------------------------------------------------------

class GetTwistSoftLimitationRequest {
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
    // Serializes a message object of type GetTwistSoftLimitationRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetTwistSoftLimitationRequest
    let len;
    let data = new GetTwistSoftLimitationRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetTwistSoftLimitationRequest';
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
    const resolved = new GetTwistSoftLimitationRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetTwistSoftLimitationResponse {
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
        this.output = new TwistLimitation();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetTwistSoftLimitationResponse
    // Serialize message field [output]
    bufferOffset = TwistLimitation.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetTwistSoftLimitationResponse
    let len;
    let data = new GetTwistSoftLimitationResponse(null);
    // Deserialize message field [output]
    data.output = TwistLimitation.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetTwistSoftLimitationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9072bb865353880c3de62e793dbcdb4e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    TwistLimitation output
    
    ================================================================================
    MSG: kortex_driver/TwistLimitation
    
    float32 linear
    float32 angular
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetTwistSoftLimitationResponse(null);
    if (msg.output !== undefined) {
      resolved.output = TwistLimitation.Resolve(msg.output)
    }
    else {
      resolved.output = new TwistLimitation()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetTwistSoftLimitationRequest,
  Response: GetTwistSoftLimitationResponse,
  md5sum() { return '69b1f43d97a46e21f8b8586022fd23ec'; },
  datatype() { return 'kortex_driver/GetTwistSoftLimitation'; }
};
