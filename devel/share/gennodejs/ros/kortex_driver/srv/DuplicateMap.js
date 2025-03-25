// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MapHandle = require('../msg/MapHandle.js');

//-----------------------------------------------------------


//-----------------------------------------------------------

class DuplicateMapRequest {
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
        this.input = new MapHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DuplicateMapRequest
    // Serialize message field [input]
    bufferOffset = MapHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DuplicateMapRequest
    let len;
    let data = new DuplicateMapRequest(null);
    // Deserialize message field [input]
    data.input = MapHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/DuplicateMapRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5bece15bd6f474817d8cf8261b2df5e4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    MapHandle input
    
    ================================================================================
    MSG: kortex_driver/MapHandle
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DuplicateMapRequest(null);
    if (msg.input !== undefined) {
      resolved.input = MapHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new MapHandle()
    }

    return resolved;
    }
};

class DuplicateMapResponse {
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
        this.output = new MapHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DuplicateMapResponse
    // Serialize message field [output]
    bufferOffset = MapHandle.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DuplicateMapResponse
    let len;
    let data = new DuplicateMapResponse(null);
    // Deserialize message field [output]
    data.output = MapHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/DuplicateMapResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '335f209b31742c233f4d4fd3cb08b30f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    MapHandle output
    
    ================================================================================
    MSG: kortex_driver/MapHandle
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DuplicateMapResponse(null);
    if (msg.output !== undefined) {
      resolved.output = MapHandle.Resolve(msg.output)
    }
    else {
      resolved.output = new MapHandle()
    }

    return resolved;
    }
};

module.exports = {
  Request: DuplicateMapRequest,
  Response: DuplicateMapResponse,
  md5sum() { return 'bfcefbce8997010c78c9f62aadaabcab'; },
  datatype() { return 'kortex_driver/DuplicateMap'; }
};
