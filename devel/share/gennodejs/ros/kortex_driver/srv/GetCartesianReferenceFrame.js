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

let CartesianReferenceFrameInfo = require('../msg/CartesianReferenceFrameInfo.js');

//-----------------------------------------------------------

class GetCartesianReferenceFrameRequest {
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
    // Serializes a message object of type GetCartesianReferenceFrameRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCartesianReferenceFrameRequest
    let len;
    let data = new GetCartesianReferenceFrameRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetCartesianReferenceFrameRequest';
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
    const resolved = new GetCartesianReferenceFrameRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetCartesianReferenceFrameResponse {
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
        this.output = new CartesianReferenceFrameInfo();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetCartesianReferenceFrameResponse
    // Serialize message field [output]
    bufferOffset = CartesianReferenceFrameInfo.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetCartesianReferenceFrameResponse
    let len;
    let data = new GetCartesianReferenceFrameResponse(null);
    // Deserialize message field [output]
    data.output = CartesianReferenceFrameInfo.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetCartesianReferenceFrameResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3b0d036926318c86039741c2c27bc6fa';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CartesianReferenceFrameInfo output
    
    ================================================================================
    MSG: kortex_driver/CartesianReferenceFrameInfo
    
    uint32 reference_frame
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetCartesianReferenceFrameResponse(null);
    if (msg.output !== undefined) {
      resolved.output = CartesianReferenceFrameInfo.Resolve(msg.output)
    }
    else {
      resolved.output = new CartesianReferenceFrameInfo()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetCartesianReferenceFrameRequest,
  Response: GetCartesianReferenceFrameResponse,
  md5sum() { return 'fd1f94f7574ce13875b4bba8e911f051'; },
  datatype() { return 'kortex_driver/GetCartesianReferenceFrame'; }
};
