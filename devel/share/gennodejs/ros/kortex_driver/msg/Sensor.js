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

class Sensor {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
    }
    else {
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Sensor
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Sensor
    let len;
    let data = new Sensor(null);
    return data;
  }

  static getMessageSize(object) {
    return 0;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/Sensor';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4dd10d615f1a44376aea67ca8eb604d0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 SENSOR_UNSPECIFIED = 0
    
    uint32 SENSOR_COLOR = 1
    
    uint32 SENSOR_DEPTH = 2
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Sensor(null);
    return resolved;
    }
};

// Constants for message
Sensor.Constants = {
  SENSOR_UNSPECIFIED: 0,
  SENSOR_COLOR: 1,
  SENSOR_DEPTH: 2,
}

module.exports = Sensor;
