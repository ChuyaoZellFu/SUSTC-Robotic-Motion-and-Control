// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MapGroupHandle = require('./MapGroupHandle.js');
let MappingHandle = require('./MappingHandle.js');
let MapHandle = require('./MapHandle.js');

//-----------------------------------------------------------

class MapGroup {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.group_handle = null;
      this.name = null;
      this.related_mapping_handle = null;
      this.parent_group_handle = null;
      this.children_map_group_handles = null;
      this.map_handles = null;
      this.application_data = null;
    }
    else {
      if (initObj.hasOwnProperty('group_handle')) {
        this.group_handle = initObj.group_handle
      }
      else {
        this.group_handle = new MapGroupHandle();
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('related_mapping_handle')) {
        this.related_mapping_handle = initObj.related_mapping_handle
      }
      else {
        this.related_mapping_handle = new MappingHandle();
      }
      if (initObj.hasOwnProperty('parent_group_handle')) {
        this.parent_group_handle = initObj.parent_group_handle
      }
      else {
        this.parent_group_handle = new MapGroupHandle();
      }
      if (initObj.hasOwnProperty('children_map_group_handles')) {
        this.children_map_group_handles = initObj.children_map_group_handles
      }
      else {
        this.children_map_group_handles = [];
      }
      if (initObj.hasOwnProperty('map_handles')) {
        this.map_handles = initObj.map_handles
      }
      else {
        this.map_handles = [];
      }
      if (initObj.hasOwnProperty('application_data')) {
        this.application_data = initObj.application_data
      }
      else {
        this.application_data = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MapGroup
    // Serialize message field [group_handle]
    bufferOffset = MapGroupHandle.serialize(obj.group_handle, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [related_mapping_handle]
    bufferOffset = MappingHandle.serialize(obj.related_mapping_handle, buffer, bufferOffset);
    // Serialize message field [parent_group_handle]
    bufferOffset = MapGroupHandle.serialize(obj.parent_group_handle, buffer, bufferOffset);
    // Serialize message field [children_map_group_handles]
    // Serialize the length for message field [children_map_group_handles]
    bufferOffset = _serializer.uint32(obj.children_map_group_handles.length, buffer, bufferOffset);
    obj.children_map_group_handles.forEach((val) => {
      bufferOffset = MapGroupHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [map_handles]
    // Serialize the length for message field [map_handles]
    bufferOffset = _serializer.uint32(obj.map_handles.length, buffer, bufferOffset);
    obj.map_handles.forEach((val) => {
      bufferOffset = MapHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [application_data]
    bufferOffset = _serializer.string(obj.application_data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MapGroup
    let len;
    let data = new MapGroup(null);
    // Deserialize message field [group_handle]
    data.group_handle = MapGroupHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [related_mapping_handle]
    data.related_mapping_handle = MappingHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [parent_group_handle]
    data.parent_group_handle = MapGroupHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [children_map_group_handles]
    // Deserialize array length for message field [children_map_group_handles]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.children_map_group_handles = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.children_map_group_handles[i] = MapGroupHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [map_handles]
    // Deserialize array length for message field [map_handles]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.map_handles = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.map_handles[i] = MapHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [application_data]
    data.application_data = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += 8 * object.children_map_group_handles.length;
    length += 8 * object.map_handles.length;
    length += _getByteLength(object.application_data);
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/MapGroup';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f19116dedc7aefc01a2fc92e64e79046';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new MapGroup(null);
    if (msg.group_handle !== undefined) {
      resolved.group_handle = MapGroupHandle.Resolve(msg.group_handle)
    }
    else {
      resolved.group_handle = new MapGroupHandle()
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.related_mapping_handle !== undefined) {
      resolved.related_mapping_handle = MappingHandle.Resolve(msg.related_mapping_handle)
    }
    else {
      resolved.related_mapping_handle = new MappingHandle()
    }

    if (msg.parent_group_handle !== undefined) {
      resolved.parent_group_handle = MapGroupHandle.Resolve(msg.parent_group_handle)
    }
    else {
      resolved.parent_group_handle = new MapGroupHandle()
    }

    if (msg.children_map_group_handles !== undefined) {
      resolved.children_map_group_handles = new Array(msg.children_map_group_handles.length);
      for (let i = 0; i < resolved.children_map_group_handles.length; ++i) {
        resolved.children_map_group_handles[i] = MapGroupHandle.Resolve(msg.children_map_group_handles[i]);
      }
    }
    else {
      resolved.children_map_group_handles = []
    }

    if (msg.map_handles !== undefined) {
      resolved.map_handles = new Array(msg.map_handles.length);
      for (let i = 0; i < resolved.map_handles.length; ++i) {
        resolved.map_handles[i] = MapHandle.Resolve(msg.map_handles[i]);
      }
    }
    else {
      resolved.map_handles = []
    }

    if (msg.application_data !== undefined) {
      resolved.application_data = msg.application_data;
    }
    else {
      resolved.application_data = ''
    }

    return resolved;
    }
};

module.exports = MapGroup;
