// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ProtectionZone = require('./ProtectionZone.js');

//-----------------------------------------------------------

class ProtectionZoneList {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.protection_zones = null;
    }
    else {
      if (initObj.hasOwnProperty('protection_zones')) {
        this.protection_zones = initObj.protection_zones
      }
      else {
        this.protection_zones = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ProtectionZoneList
    // Serialize message field [protection_zones]
    // Serialize the length for message field [protection_zones]
    bufferOffset = _serializer.uint32(obj.protection_zones.length, buffer, bufferOffset);
    obj.protection_zones.forEach((val) => {
      bufferOffset = ProtectionZone.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ProtectionZoneList
    let len;
    let data = new ProtectionZoneList(null);
    // Deserialize message field [protection_zones]
    // Deserialize array length for message field [protection_zones]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.protection_zones = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.protection_zones[i] = ProtectionZone.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.protection_zones.forEach((val) => {
      length += ProtectionZone.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/ProtectionZoneList';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a08108637290aeb7551436f88ea3c88f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    ProtectionZone[] protection_zones
    ================================================================================
    MSG: kortex_driver/ProtectionZone
    
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
    const resolved = new ProtectionZoneList(null);
    if (msg.protection_zones !== undefined) {
      resolved.protection_zones = new Array(msg.protection_zones.length);
      for (let i = 0; i < resolved.protection_zones.length; ++i) {
        resolved.protection_zones[i] = ProtectionZone.Resolve(msg.protection_zones[i]);
      }
    }
    else {
      resolved.protection_zones = []
    }

    return resolved;
    }
};

module.exports = ProtectionZoneList;
