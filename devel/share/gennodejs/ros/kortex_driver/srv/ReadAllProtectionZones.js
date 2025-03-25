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

let ProtectionZoneList = require('../msg/ProtectionZoneList.js');

//-----------------------------------------------------------

class ReadAllProtectionZonesRequest {
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
    // Serializes a message object of type ReadAllProtectionZonesRequest
    // Serialize message field [input]
    bufferOffset = Empty.serialize(obj.input, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadAllProtectionZonesRequest
    let len;
    let data = new ReadAllProtectionZonesRequest(null);
    // Deserialize message field [input]
    data.input = Empty.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ReadAllProtectionZonesRequest';
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
    const resolved = new ReadAllProtectionZonesRequest(null);
    if (msg.input !== undefined) {
      resolved.input = Empty.Resolve(msg.input)
    }
    else {
      resolved.input = new Empty()
    }

    return resolved;
    }
};

class ReadAllProtectionZonesResponse {
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
        this.output = new ProtectionZoneList();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ReadAllProtectionZonesResponse
    // Serialize message field [output]
    bufferOffset = ProtectionZoneList.serialize(obj.output, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ReadAllProtectionZonesResponse
    let len;
    let data = new ReadAllProtectionZonesResponse(null);
    // Deserialize message field [output]
    data.output = ProtectionZoneList.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ProtectionZoneList.getMessageSize(object.output);
    return length;
  }

  static datatype() {
    // Returns string type for a service object
    return 'kortex_driver/ReadAllProtectionZonesResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8334fb0dbc5a4fdae35c676ee015197';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ProtectionZoneList output
    
    ================================================================================
    MSG: kortex_driver/ProtectionZoneList
    
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
    const resolved = new ReadAllProtectionZonesResponse(null);
    if (msg.output !== undefined) {
      resolved.output = ProtectionZoneList.Resolve(msg.output)
    }
    else {
      resolved.output = new ProtectionZoneList()
    }

    return resolved;
    }
};

module.exports = {
  Request: ReadAllProtectionZonesRequest,
  Response: ReadAllProtectionZonesResponse,
  md5sum() { return '26bc18c08c1b8389444ea1d92e6c8745'; },
  datatype() { return 'kortex_driver/ReadAllProtectionZones'; }
};
