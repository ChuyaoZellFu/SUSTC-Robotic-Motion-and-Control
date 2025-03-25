// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ProtectionZoneHandle = require('./ProtectionZoneHandle.js');
let ZoneShape = require('./ZoneShape.js');
let CartesianLimitation = require('./CartesianLimitation.js');

//-----------------------------------------------------------

class ProtectionZone {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.handle = null;
      this.name = null;
      this.application_data = null;
      this.is_enabled = null;
      this.shape = null;
      this.limitations = null;
      this.envelope_limitations = null;
    }
    else {
      if (initObj.hasOwnProperty('handle')) {
        this.handle = initObj.handle
      }
      else {
        this.handle = new ProtectionZoneHandle();
      }
      if (initObj.hasOwnProperty('name')) {
        this.name = initObj.name
      }
      else {
        this.name = '';
      }
      if (initObj.hasOwnProperty('application_data')) {
        this.application_data = initObj.application_data
      }
      else {
        this.application_data = '';
      }
      if (initObj.hasOwnProperty('is_enabled')) {
        this.is_enabled = initObj.is_enabled
      }
      else {
        this.is_enabled = false;
      }
      if (initObj.hasOwnProperty('shape')) {
        this.shape = initObj.shape
      }
      else {
        this.shape = new ZoneShape();
      }
      if (initObj.hasOwnProperty('limitations')) {
        this.limitations = initObj.limitations
      }
      else {
        this.limitations = [];
      }
      if (initObj.hasOwnProperty('envelope_limitations')) {
        this.envelope_limitations = initObj.envelope_limitations
      }
      else {
        this.envelope_limitations = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ProtectionZone
    // Serialize message field [handle]
    bufferOffset = ProtectionZoneHandle.serialize(obj.handle, buffer, bufferOffset);
    // Serialize message field [name]
    bufferOffset = _serializer.string(obj.name, buffer, bufferOffset);
    // Serialize message field [application_data]
    bufferOffset = _serializer.string(obj.application_data, buffer, bufferOffset);
    // Serialize message field [is_enabled]
    bufferOffset = _serializer.bool(obj.is_enabled, buffer, bufferOffset);
    // Serialize message field [shape]
    bufferOffset = ZoneShape.serialize(obj.shape, buffer, bufferOffset);
    // Serialize message field [limitations]
    // Serialize the length for message field [limitations]
    bufferOffset = _serializer.uint32(obj.limitations.length, buffer, bufferOffset);
    obj.limitations.forEach((val) => {
      bufferOffset = CartesianLimitation.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [envelope_limitations]
    // Serialize the length for message field [envelope_limitations]
    bufferOffset = _serializer.uint32(obj.envelope_limitations.length, buffer, bufferOffset);
    obj.envelope_limitations.forEach((val) => {
      bufferOffset = CartesianLimitation.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ProtectionZone
    let len;
    let data = new ProtectionZone(null);
    // Deserialize message field [handle]
    data.handle = ProtectionZoneHandle.deserialize(buffer, bufferOffset);
    // Deserialize message field [name]
    data.name = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [application_data]
    data.application_data = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [is_enabled]
    data.is_enabled = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [shape]
    data.shape = ZoneShape.deserialize(buffer, bufferOffset);
    // Deserialize message field [limitations]
    // Deserialize array length for message field [limitations]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.limitations = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.limitations[i] = CartesianLimitation.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [envelope_limitations]
    // Deserialize array length for message field [envelope_limitations]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.envelope_limitations = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.envelope_limitations[i] = CartesianLimitation.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.name);
    length += _getByteLength(object.application_data);
    length += ZoneShape.getMessageSize(object.shape);
    length += 12 * object.limitations.length;
    length += 12 * object.envelope_limitations.length;
    return length + 25;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ProtectionZone';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9648b3590ddbe012236cdc588d3f6ef8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    ProtectionZoneHandle handle
    string name
    string application_data
    bool is_enabled
    ZoneShape shape
    CartesianLimitation[] limitations
    CartesianLimitation[] envelope_limitations
    ================================================================================
    MSG: kortex_driver/ProtectionZoneHandle
    
    uint32 identifier
    uint32 permission
    ================================================================================
    MSG: kortex_driver/ZoneShape
    
    uint32 shape_type
    Point origin
    Base_RotationMatrix orientation
    float32[] dimensions
    float32 envelope_thickness
    ================================================================================
    MSG: kortex_driver/Point
    
    float32 x
    float32 y
    float32 z
    ================================================================================
    MSG: kortex_driver/Base_RotationMatrix
    
    Base_RotationMatrixRow row1
    Base_RotationMatrixRow row2
    Base_RotationMatrixRow row3
    ================================================================================
    MSG: kortex_driver/Base_RotationMatrixRow
    
    float32 column1
    float32 column2
    float32 column3
    ================================================================================
    MSG: kortex_driver/CartesianLimitation
    
    uint32 type
    float32 translation
    float32 orientation
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ProtectionZone(null);
    if (msg.handle !== undefined) {
      resolved.handle = ProtectionZoneHandle.Resolve(msg.handle)
    }
    else {
      resolved.handle = new ProtectionZoneHandle()
    }

    if (msg.name !== undefined) {
      resolved.name = msg.name;
    }
    else {
      resolved.name = ''
    }

    if (msg.application_data !== undefined) {
      resolved.application_data = msg.application_data;
    }
    else {
      resolved.application_data = ''
    }

    if (msg.is_enabled !== undefined) {
      resolved.is_enabled = msg.is_enabled;
    }
    else {
      resolved.is_enabled = false
    }

    if (msg.shape !== undefined) {
      resolved.shape = ZoneShape.Resolve(msg.shape)
    }
    else {
      resolved.shape = new ZoneShape()
    }

    if (msg.limitations !== undefined) {
      resolved.limitations = new Array(msg.limitations.length);
      for (let i = 0; i < resolved.limitations.length; ++i) {
        resolved.limitations[i] = CartesianLimitation.Resolve(msg.limitations[i]);
      }
    }
    else {
      resolved.limitations = []
    }

    if (msg.envelope_limitations !== undefined) {
      resolved.envelope_limitations = new Array(msg.envelope_limitations.length);
      for (let i = 0; i < resolved.envelope_limitations.length; ++i) {
        resolved.envelope_limitations[i] = CartesianLimitation.Resolve(msg.envelope_limitations[i]);
      }
    }
    else {
      resolved.envelope_limitations = []
    }

    return resolved;
    }
};

module.exports = ProtectionZone;
