// Auto-generated. Do not edit!

// (in-package kortex_driver.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Unit {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Unit
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Unit
    let len;
    let data = new Unit(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Unit';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '38eeb5ea9959fe1af60e856b88a5f9a7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 UNSPECIFIED_UNIT = 0
    
    uint32 CELSIUS = 1
    
    uint32 AMPERE = 2
    
    uint32 VOLT = 3
    
    uint32 METER_PER_SECOND = 4
    
    uint32 DEGREE_PER_SECOND = 5
    
    uint32 METER_PER_SECOND_2 = 6
    
    uint32 DEGREE_PER_SECOND_2 = 7
    
    uint32 NEWTON = 8
    
    uint32 NEWTON_METER = 9
    
    uint32 KILOGRAM = 10
    
    uint32 DEGREE = 11
    
    uint32 TICK = 12
    
    uint32 DEGREE_PER_MILLISECOND = 13
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Unit(null);
    return resolved;
    }
};

// Constants for message
Unit.Constants = {
  UNSPECIFIED_UNIT: 0,
  CELSIUS: 1,
  AMPERE: 2,
  VOLT: 3,
  METER_PER_SECOND: 4,
  DEGREE_PER_SECOND: 5,
  METER_PER_SECOND_2: 6,
  DEGREE_PER_SECOND_2: 7,
  NEWTON: 8,
  NEWTON_METER: 9,
  KILOGRAM: 10,
  DEGREE: 11,
  TICK: 12,
  DEGREE_PER_MILLISECOND: 13,
}

module.exports = Unit;
