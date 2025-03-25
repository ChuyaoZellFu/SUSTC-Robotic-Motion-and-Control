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

class OptionIdentifier {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sensor = null;
      this.option = null;
    }
    else {
      if (initObj.hasOwnProperty('sensor')) {
        this.sensor = initObj.sensor
      }
      else {
        this.sensor = 0;
      }
      if (initObj.hasOwnProperty('option')) {
        this.option = initObj.option
      }
      else {
        this.option = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OptionIdentifier
    // Serialize message field [sensor]
    bufferOffset = _serializer.uint32(obj.sensor, buffer, bufferOffset);
    // Serialize message field [option]
    bufferOffset = _serializer.uint32(obj.option, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OptionIdentifier
    let len;
    let data = new OptionIdentifier(null);
    // Deserialize message field [sensor]
    data.sensor = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [option]
    data.option = _deserializer.uint32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/OptionIdentifier';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '23d53d4ecb95c9409f8e27d3bfff4aca';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 sensor
    uint32 option
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OptionIdentifier(null);
    if (msg.sensor !== undefined) {
      resolved.sensor = msg.sensor;
    }
    else {
      resolved.sensor = 0
    }

    if (msg.option !== undefined) {
      resolved.option = msg.option;
    }
    else {
      resolved.option = 0
    }

    return resolved;
    }
};

module.exports = OptionIdentifier;
