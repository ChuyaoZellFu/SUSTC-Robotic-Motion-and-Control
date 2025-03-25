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

let MappingList = require('../msg/MappingList.js');

//-----------------------------------------------------------

class ReadAllMappingsRequest {
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
    // Serializes a message object of type ReadAllMappingsRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadAllMappingsRequest
    let len;
    let data = new ReadAllMappingsRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ReadAllMappingsRequest';
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
    const resolved = new ReadAllMappingsRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class ReadAllMappingsResponse {
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
        this.output = new MappingList();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReadAllMappingsResponse
    // Serialize message field [output]
    bufferOffset = MappingList.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadAllMappingsResponse
    let len;
    let data = new ReadAllMappingsResponse(null);
    // Deserialize message field [output]
    data.output = MappingList.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += MappingList.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ReadAllMappingsResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f318987fb4be88e1c786ff4d68235a14';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    MappingList output
    
    ================================================================================
    MSG: kortex_driver/MappingList
    
    Mapping[] mappings
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
    const resolved = new ReadAllMappingsResponse(null);
    if (msg.output !== undefined) {
      resolved.output = MappingList.Resolve(msg.output)
    }
    else {
      resolved.output = new MappingList()
    }

    return resolved;
    }
};

module.exports = {
  Request: ReadAllMappingsRequest,
  Response: ReadAllMappingsResponse,
  md5sum() { return 'b5feb1f35cacb5c11e26808022e25f5c'; },
  datatype() { return 'kortex_driver/ReadAllMappings'; }
};
