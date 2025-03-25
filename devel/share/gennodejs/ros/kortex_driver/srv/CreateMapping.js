// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Mapping = require('../msg/Mapping.js');

//-----------------------------------------------------------

let MappingHandle = require('../msg/MappingHandle.js');

//-----------------------------------------------------------

class CreateMappingRequest {
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
        this.input = new Mapping();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CreateMappingRequest
    // Serialize message field [input]
    bufferOffset = Mapping.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CreateMappingRequest
    let len;
    let data = new CreateMappingRequest(null);
    // Deserialize message field [input]
    data.input = Mapping.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += Mapping.getMessageSize(object.input);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/CreateMappingRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b5bb1290f598f97b2beeda37713172c1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Mapping input
    
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
    const resolved = new CreateMappingRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Mapping.Resolve(msg.input)
    }
    else {
      resolved.input = new Mapping()
    }

    return resolved;
    }
};

class CreateMappingResponse {
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
    // Serializes a message object of type CreateMappingResponse
    // Serialize message field [output]
    bufferOffset = MappingHandle.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CreateMappingResponse
    let len;
    let data = new CreateMappingResponse(null);
    // Deserialize message field [output]
    data.output = MappingHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/CreateMappingResponse';
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
    const resolved = new CreateMappingResponse(null);
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
  Request: CreateMappingRequest,
  Response: CreateMappingResponse,
  md5sum() { return '1672eeb2532b125a8c2350096e2628b8'; },
  datatype() { return 'kortex_driver/CreateMapping'; }
};
