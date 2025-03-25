// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ControllerHandle = require('./ControllerHandle.js');
let MappingHandle = require('./MappingHandle.js');

//-----------------------------------------------------------

class ControllerConfiguration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle = null;
      this.name = null;
      this.active_mapping_handle = null;
      this.analog_input_identifier_enum = null;
      this.digital_input_identifier_enum = null;
    }
    else {
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new ControllerHandle();
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('active_mapping_handle')) {
        this.active_mapping_handle = initObj.active_mapping_handle
      }
      else {
        this.active_mapping_handle = new MappingHandle();
      }
      if (initObj.hasOwnProperty('analog_input_identifier_enum')) {
        this.analog_input_identifier_enum = initObj.analog_input_identifier_enum
      }
      else {
        this.analog_input_identifier_enum = '';
      }
      if (initObj.hasOwnProperty('digital_input_identifier_enum')) {
        this.digital_input_identifier_enum = initObj.digital_input_identifier_enum
      }
      else {
        this.digital_input_identifier_enum = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControllerConfiguration
    // Serialize message field [handle]
    bufferOffset = ControllerHandle.serialize(obj.handle, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [active_mapping_handle]
    bufferOffset = MappingHandle.serialize(obj.active_mapping_handle, buffer, bufferOffset);
    // Serialize message field [analog_input_identifier_enum]
    bufferOffset = _serializer.string(obj.analog_input_identifier_enum, buffer, bufferOffset);
    // Serialize message field [digital_input_identifier_enum]
    bufferOffset = _serializer.string(obj.digital_input_identifier_enum, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControllerConfiguration
    let len;
    let data = new ControllerConfiguration(null);
    // Deserialize message field [handle]
    data.handle = ControllerHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [active_mapping_handle]
    data.active_mapping_handle = MappingHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [analog_input_identifier_enum]
    data.analog_input_identifier_enum = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [digital_input_identifier_enum]
    data.digital_input_identifier_enum = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += _getByteLength(object.analog_input_identifier_enum);
    length += _getByteLength(object.digital_input_identifier_enum);
    return length + 28;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ControllerConfiguration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0a2e41d50fc9c491b9a0c9000a90ca85';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    ControllerHandle handle
    string name
    MappingHandle active_mapping_handle
    string analog_input_identifier_enum
    string digital_input_identifier_enum
    ================================================================================
    MSG: kortex_driver/ControllerHandle
    
    uint32 type
    uint32 controller_identifier
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
    const resolved = new ControllerConfiguration(null);
    if (msg.handle !== undefined) {
      resolved.handle = ControllerHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new ControllerHandle()
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.active_mapping_handle !== undefined) {
      resolved.active_mapping_handle = MappingHandle.Resolve(msg.active_mapping_handle)
    }
    else {
      resolved.active_mapping_handle = new MappingHandle()
    }

    if (msg.analog_input_identifier_enum !== undefined) {
      resolved.analog_input_identifier_enum = msg.analog_input_identifier_enum;
    }
    else {
      resolved.analog_input_identifier_enum = ''
    }

    if (msg.digital_input_identifier_enum !== undefined) {
      resolved.digital_input_identifier_enum = msg.digital_input_identifier_enum;
    }
    else {
      resolved.digital_input_identifier_enum = ''
    }

    return resolved;
    }
};

module.exports = ControllerConfiguration;
