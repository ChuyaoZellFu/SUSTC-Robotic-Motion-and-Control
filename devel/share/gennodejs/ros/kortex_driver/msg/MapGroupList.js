// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MapGroup = require('./MapGroup.js');

//-----------------------------------------------------------

class MapGroupList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.map_groups = null;
    }
    else {
      if (initObj.hasOwnProperty('map_groups')) {
        this.map_groups = initObj.map_groups
      }
      else {
        this.map_groups = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MapGroupList
    // Serialize message field [map_groups]
    // Serialize the length for message field [map_groups]
    bufferOffset = _serializer.uint32(obj.map_groups.length, buffer, bufferOffset);
    obj.map_groups.forEach((val) => {
      bufferOffset = MapGroup.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MapGroupList
    let len;
    let data = new MapGroupList(null);
    // Deserialize message field [map_groups]
    // Deserialize array length for message field [map_groups]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.map_groups = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.map_groups[i] = MapGroup.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.map_groups.forEach((val) => {
      length += MapGroup.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/MapGroupList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '60049e6988ccab2e80e3d3cdf02a81da';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    MapGroup[] map_groups
    ================================================================================
    MSG: kortex_driver/MapGroup
    
    MapGroupHandle group_handle
    string name
    MappingHandle related_mapping_handle
    MapGroupHandle parent_group_handle
    MapGroupHandle[] children_map_group_handles
    MapHandle[] map_handles
    string application_data
    ================================================================================
    MSG: kortex_driver/MapGroupHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/MappingHandle
    
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
    const resolved = new MapGroupList(null);
    if (msg.map_groups !== undefined) {
      resolved.map_groups = new Array(msg.map_groups.length);
      for (let i = 0; i < resolved.map_groups.length; ++i) {
        resolved.map_groups[i] = MapGroup.Resolve(msg.map_groups[i]);
      }
    }
    else {
      resolved.map_groups = []
    }

    return resolved;
    }
};

module.exports = MapGroupList;
