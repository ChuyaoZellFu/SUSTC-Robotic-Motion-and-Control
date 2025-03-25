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

let Mapping = require('../msg/Mapping.js');

//-----------------------------------------------------------

class ReadMappingRequest {
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
    // Serializes a message object of type ReadMappingRequest
    // Serialize message field [input]
    bufferOffset = MappingHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadMappingRequest
    let len;
    let data = new ReadMappingRequest(null);
    // Deserialize message field [input]
    data.input = MappingHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ReadMappingRequest';
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
    const resolved = new ReadMappingRequest(null);
    if (msg.input !== undefined) {
      resolved.input = MappingHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new MappingHandle()
    }

    return resolved;
    }
};

class ReadMappingResponse {
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
        this.output = new Mapping();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReadMappingResponse
    // Serialize message field [output]
    bufferOffset = Mapping.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadMappingResponse
    let len;
    let data = new ReadMappingResponse(null);
    // Deserialize message field [output]
    data.output = Mapping.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Mapping.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ReadMappingResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b0531d255596ca4470f629d3fa3c3f8e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Mapping output
    
    ================================================================================
    MSG: kortex_driver/Mapping
    
    MappingHandle handle
    string name
    uint32 controller_identifier
    MapGroupHandle active_map_group_handle
    MapGroupHandle[] map_group_handles
    MapHandle active_map_handle
    MapHandle[] map_handles
    string application_data
    ================================================================================
    MSG: kortex_driver/MappingHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/MapGroupHandle
    
    uint32 identifier
    uint32 permission
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
    const resolved = new ReadMappingResponse(null);
    if (msg.output !== undefined) {
      resolved.output = Mapping.Resolve(msg.output)
    }
    else {
      resolved.output = new Mapping()
    }

    return resolved;
    }
};

module.exports = {
  Request: ReadMappingRequest,
  Response: ReadMappingResponse,
  md5sum() { return 'a098ab486d258af0a3206d3f4ae9d15c'; },
  datatype() { return 'kortex_driver/ReadMapping'; }
};
