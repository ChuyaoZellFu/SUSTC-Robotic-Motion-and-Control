// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MappingHandle = require('../msg/MappingHandle.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class DuplicateMappingRequest {
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
        this.input = new MappingHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DuplicateMappingRequest
    // Serialize message field [input]
    bufferOffset = MappingHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DuplicateMappingRequest
    let len;
    let data = new DuplicateMappingRequest(null);
    // Deserialize message field [input]
    data.input = MappingHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/DuplicateMappingRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5bece15bd6f474817d8cf8261b2df5e4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    MappingHandle input
    
    ================================================================================
    MSG: kortex_driver/MappingHandle
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DuplicateMappingRequest(null);
    if (msg.input !== undefined) {
      resolved.input = MappingHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new MappingHandle()
    }

    return resolved;
    }
};

class DuplicateMappingResponse {
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
        this.output = new MappingHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DuplicateMappingResponse
    // Serialize message field [output]
    bufferOffset = MappingHandle.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DuplicateMappingResponse
    let len;
    let data = new DuplicateMappingResponse(null);
    // Deserialize message field [output]
    data.output = MappingHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/DuplicateMappingResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '335f209b31742c233f4d4fd3cb08b30f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    MappingHandle output
    
    ================================================================================
    MSG: kortex_driver/MappingHandle
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DuplicateMappingResponse(null);
    if (msg.output !== undefined) {
      resolved.output = MappingHandle.Resolve(msg.output)
    }
    else {
      resolved.output = new MappingHandle()
    }

    return resolved;
    }
};

module.exports = {
  Request: DuplicateMappingRequest,
  Response: DuplicateMappingResponse,
  md5sum() { return 'bfcefbce8997010c78c9f62aadaabcab'; },
  datatype() { return 'kortex_driver/DuplicateMapping'; }
};
