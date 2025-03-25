// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MappingHandle = require('./MappingHandle.js');
let MapGroupHandle = require('./MapGroupHandle.js');
let MapHandle = require('./MapHandle.js');

//-----------------------------------------------------------

class Mapping {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle = null;
      this.name = null;
      this.controller_identifier = null;
      this.active_map_group_handle = null;
      this.map_group_handles = null;
      this.active_map_handle = null;
      this.map_handles = null;
      this.application_data = null;
    }
    else {
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new MappingHandle();
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('controller_identifier')) {
        this.controller_identifier = initObj.controller_identifier
      }
      else {
        this.controller_identifier = 0;
      }
      if (initObj.hasOwnProperty('active_map_group_handle')) {
        this.active_map_group_handle = initObj.active_map_group_handle
      }
      else {
        this.active_map_group_handle = new MapGroupHandle();
      }
      if (initObj.hasOwnProperty('map_group_handles')) {
        this.map_group_handles = initObj.map_group_handles
      }
      else {
        this.map_group_handles = [];
      }
      if (initObj.hasOwnProperty('active_map_handle')) {
        this.active_map_handle = initObj.active_map_handle
      }
      else {
        this.active_map_handle = new MapHandle();
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
    // Serializes a message object of type Mapping
    // Serialize message field [handle]
    bufferOffset = MappingHandle.serialize(obj.handle, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [controller_identifier]
    bufferOffset = _serializer.uint32(obj.controller_identifier, buffer, bufferOffset);
    // Serialize message field [active_map_group_handle]
    bufferOffset = MapGroupHandle.serialize(obj.active_map_group_handle, buffer, bufferOffset);
    // Serialize message field [map_group_handles]
    // Serialize the length for message field [map_group_handles]
    bufferOffset = _serializer.uint32(obj.map_group_handles.length, buffer, bufferOffset);
    obj.map_group_handles.forEach((val) => {
      bufferOffset = MapGroupHandle.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [active_map_handle]
    bufferOffset = MapHandle.serialize(obj.active_map_handle, buffer, bufferOffset);
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
    //deserializes a message object of type Mapping
    let len;
    let data = new Mapping(null);
    // Deserialize message field [handle]
    data.handle = MappingHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [controller_identifier]
    data.controller_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [active_map_group_handle]
    data.active_map_group_handle = MapGroupHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [map_group_handles]
    // Deserialize array length for message field [map_group_handles]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.map_group_handles = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.map_group_handles[i] = MapGroupHandle.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [active_map_handle]
    data.active_map_handle = MapHandle.deserialize(buffer, bufferOffset);
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
    length += 8 * object.map_group_handles.length;
    length += 8 * object.map_handles.length;
    length += _getByteLength(object.application_data);
    return length + 44;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Mapping';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fda005d3531fdfb82d459bb0078389d2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
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
    const resolved = new Mapping(null);
    if (msg.handle !== undefined) {
      resolved.handle = MappingHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new MappingHandle()
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.controller_identifier !== undefined) {
      resolved.controller_identifier = msg.controller_identifier;
    }
    else {
      resolved.controller_identifier = 0
    }

    if (msg.active_map_group_handle !== undefined) {
      resolved.active_map_group_handle = MapGroupHandle.Resolve(msg.active_map_group_handle)
    }
    else {
      resolved.active_map_group_handle = new MapGroupHandle()
    }

    if (msg.map_group_handles !== undefined) {
      resolved.map_group_handles = new Array(msg.map_group_handles.length);
      for (let i = 0; i < resolved.map_group_handles.length; ++i) {
        resolved.map_group_handles[i] = MapGroupHandle.Resolve(msg.map_group_handles[i]);
      }
    }
    else {
      resolved.map_group_handles = []
    }

    if (msg.active_map_handle !== undefined) {
      resolved.active_map_handle = MapHandle.Resolve(msg.active_map_handle)
    }
    else {
      resolved.active_map_handle = new MapHandle()
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

module.exports = Mapping;
