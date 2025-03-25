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
let MapHandle = require('./MapHandle.js');

//-----------------------------------------------------------

class SwitchControlMapping {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.controller_identifier = null;
      this.map_group_handle = null;
      this.map_handle = null;
    }
    else {
      if (initObj.hasOwnProperty('controller_identifier')) {
        this.controller_identifier = initObj.controller_identifier
      }
      else {
        this.controller_identifier = 0;
      }
      if (initObj.hasOwnProperty('map_group_handle')) {
        this.map_group_handle = initObj.map_group_handle
      }
      else {
        this.map_group_handle = new MapGroupHandle();
      }
      if (initObj.hasOwnProperty('map_handle')) {
        this.map_handle = initObj.map_handle
      }
      else {
        this.map_handle = new MapHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SwitchControlMapping
    // Serialize message field [controller_identifier]
    bufferOffset = _serializer.uint32(obj.controller_identifier, buffer, bufferOffset);
    // Serialize message field [map_group_handle]
    bufferOffset = MapGroupHandle.serialize(obj.map_group_handle, buffer, bufferOffset);
    // Serialize message field [map_handle]
    bufferOffset = MapHandle.serialize(obj.map_handle, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SwitchControlMapping
    let len;
    let data = new SwitchControlMapping(null);
    // Deserialize message field [controller_identifier]
    data.controller_identifier = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [map_group_handle]
    data.map_group_handle = MapGroupHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [map_handle]
    data.map_handle = MapHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/SwitchControlMapping';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4a7c904b5b1279f1ceda6c334529fa90';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 controller_identifier
    MapGroupHandle map_group_handle
    MapHandle map_handle
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
    const resolved = new SwitchControlMapping(null);
    if (msg.controller_identifier !== undefined) {
      resolved.controller_identifier = msg.controller_identifier;
    }
    else {
      resolved.controller_identifier = 0
    }

    if (msg.map_group_handle !== undefined) {
      resolved.map_group_handle = MapGroupHandle.Resolve(msg.map_group_handle)
    }
    else {
      resolved.map_group_handle = new MapGroupHandle()
    }

    if (msg.map_handle !== undefined) {
      resolved.map_handle = MapHandle.Resolve(msg.map_handle)
    }
    else {
      resolved.map_handle = new MapHandle()
    }

    return resolved;
    }
};

module.exports = SwitchControlMapping;
