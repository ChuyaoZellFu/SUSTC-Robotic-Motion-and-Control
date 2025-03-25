// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Mapping = require('./Mapping.js');

//-----------------------------------------------------------

class MappingList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mappings = null;
    }
    else {
      if (initObj.hasOwnProperty('mappings')) {
        this.mappings = initObj.mappings
      }
      else {
        this.mappings = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MappingList
    // Serialize message field [mappings]
    // Serialize the length for message field [mappings]
    bufferOffset = _serializer.uint32(obj.mappings.length, buffer, bufferOffset);
    obj.mappings.forEach((val) => {
      bufferOffset = Mapping.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MappingList
    let len;
    let data = new MappingList(null);
    // Deserialize message field [mappings]
    // Deserialize array length for message field [mappings]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.mappings = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.mappings[i] = Mapping.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.mappings.forEach((val) => {
      length += Mapping.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/MappingList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'bb960bc69e45bc26dd9e7ed2dbaeeaaf';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new MappingList(null);
    if (msg.mappings !== undefined) {
      resolved.mappings = new Array(msg.mappings.length);
      for (let i = 0; i < resolved.mappings.length; ++i) {
        resolved.mappings[i] = Mapping.Resolve(msg.mappings[i]);
      }
    }
    else {
      resolved.mappings = []
    }

    return resolved;
    }
};

module.exports = MappingList;
