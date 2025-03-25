// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let CartesianReferenceFrameInfo = require('../msg/CartesianReferenceFrameInfo.js');

//-----------------------------------------------------------

let Empty = require('../msg/Empty.js');

//-----------------------------------------------------------

class SetCartesianReferenceFrameRequest {
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
        this.input = new CartesianReferenceFrameInfo();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCartesianReferenceFrameRequest
    // Serialize message field [input]
    bufferOffset = CartesianReferenceFrameInfo.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCartesianReferenceFrameRequest
    let len;
    let data = new SetCartesianReferenceFrameRequest(null);
    // Deserialize message field [input]
    data.input = CartesianReferenceFrameInfo.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetCartesianReferenceFrameRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ed00434fa2dc2fef05ed002df99e779f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    CartesianReferenceFrameInfo input
    
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
    const resolved = new SetCartesianReferenceFrameRequest(null);
    if (msg.input !== undefined) {
      resolved.input = CartesianReferenceFrameInfo.Resolve(msg.input)
    }
    else {
      resolved.input = new CartesianReferenceFrameInfo()
    }

    return resolved;
    }
};

class SetCartesianReferenceFrameResponse {
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
        this.output = new Empty();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetCartesianReferenceFrameResponse
    // Serialize message field [output]
    bufferOffset = Empty.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetCartesianReferenceFrameResponse
    let len;
    let data = new SetCartesianReferenceFrameResponse(null);
    // Deserialize message field [output]
    data.output = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/SetCartesianReferenceFrameResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'c6c43d221c810050f75091660f63b0cd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Empty output
    
    ================================================================================
    MSG: kortex_driver/Empty
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetCartesianReferenceFrameResponse(null);
    if (msg.output !== undefined) {
      resolved.output = Empty.Resolve(msg.output)
    }
    else {
      resolved.output = new Empty()
    }

    return resolved;
    }
};

module.exports = {
  Request: SetCartesianReferenceFrameRequest,
  Response: SetCartesianReferenceFrameResponse,
  md5sum() { return '80417b91d14dfe7b5a3f569e00b74176'; },
  datatype() { return 'kortex_driver/SetCartesianReferenceFrame'; }
};
