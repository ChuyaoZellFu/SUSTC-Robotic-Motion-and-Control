// Auto-generated. Do not edit!

// (in-package kortex_driver.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ProtectionZoneHandle = require('../msg/ProtectionZoneHandle.js');

//-----------------------------------------------------------

let ProtectionZone = require('../msg/ProtectionZone.js');

//-----------------------------------------------------------

class ReadProtectionZoneRequest {
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
        this.input = new ProtectionZoneHandle();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReadProtectionZoneRequest
    // Serialize message field [input]
    bufferOffset = ProtectionZoneHandle.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadProtectionZoneRequest
    let len;
    let data = new ReadProtectionZoneRequest(null);
    // Deserialize message field [input]
    data.input = ProtectionZoneHandle.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ReadProtectionZoneRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5bece15bd6f474817d8cf8261b2df5e4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ProtectionZoneHandle input
    
    ================================================================================
    MSG: kortex_driver/ProtectionZoneHandle
    
    uint32 identifier
    uint32 permission
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ReadProtectionZoneRequest(null);
    if (msg.input !== undefined) {
      resolved.input = ProtectionZoneHandle.Resolve(msg.input)
    }
    else {
      resolved.input = new ProtectionZoneHandle()
    }

    return resolved;
    }
};

class ReadProtectionZoneResponse {
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
        this.output = new ProtectionZone();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReadProtectionZoneResponse
    // Serialize message field [output]
    bufferOffset = ProtectionZone.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadProtectionZoneResponse
    let len;
    let data = new ReadProtectionZoneResponse(null);
    // Deserialize message field [output]
    data.output = ProtectionZone.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ProtectionZone.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ReadProtectionZoneResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '561afac1699873ff32f40f4c3927ed43';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ProtectionZone output
    
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
    const resolved = new ReadProtectionZoneResponse(null);
    if (msg.output !== undefined) {
      resolved.output = ProtectionZone.Resolve(msg.output)
    }
    else {
      resolved.output = new ProtectionZone()
    }

    return resolved;
    }
};

module.exports = {
  Request: ReadProtectionZoneRequest,
  Response: ReadProtectionZoneResponse,
  md5sum() { return 'fd53b25b5f35cd926da1c19403c70bc5'; },
  datatype() { return 'kortex_driver/ReadProtectionZone'; }
};
