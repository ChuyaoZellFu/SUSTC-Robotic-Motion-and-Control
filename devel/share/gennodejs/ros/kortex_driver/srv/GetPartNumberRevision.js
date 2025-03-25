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

let PartNumberRevision = require('../msg/PartNumberRevision.js');

//-----------------------------------------------------------

class GetPartNumberRevisionRequest {
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
    // Serializes a message object of type GetPartNumberRevisionRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPartNumberRevisionRequest
    let len;
    let data = new GetPartNumberRevisionRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetPartNumberRevisionRequest';
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
    const resolved = new GetPartNumberRevisionRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class GetPartNumberRevisionResponse {
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
        this.output = new PartNumberRevision();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GetPartNumberRevisionResponse
    // Serialize message field [output]
    bufferOffset = PartNumberRevision.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GetPartNumberRevisionResponse
    let len;
    let data = new GetPartNumberRevisionResponse(null);
    // Deserialize message field [output]
    data.output = PartNumberRevision.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += PartNumberRevision.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/GetPartNumberRevisionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6ed7d5205deef778939c240d3a8dd61a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    PartNumberRevision output
    
    ================================================================================
    MSG: kortex_driver/PartNumberRevision
    
    string part_number_revision
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GetPartNumberRevisionResponse(null);
    if (msg.output !== undefined) {
      resolved.output = PartNumberRevision.Resolve(msg.output)
    }
    else {
      resolved.output = new PartNumberRevision()
    }

    return resolved;
    }
};

module.exports = {
  Request: GetPartNumberRevisionRequest,
  Response: GetPartNumberRevisionResponse,
  md5sum() { return 'a25670b2402eb62859a04b7bd6a88b8c'; },
  datatype() { return 'kortex_driver/GetPartNumberRevision'; }
};
