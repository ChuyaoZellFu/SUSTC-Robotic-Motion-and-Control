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

class GpioPinConfiguration {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pin_id = null;
      this.pin_property = null;
      this.output_enable = null;
      this.default_output_value = null;
    }
    else {
      if (initObj.hasOwnProperty('pin_id')) {
        this.pin_id = initObj.pin_id
      }
      else {
        this.pin_id = 0;
      }
      if (initObj.hasOwnProperty('pin_property')) {
        this.pin_property = initObj.pin_property
      }
      else {
        this.pin_property = 0;
      }
      if (initObj.hasOwnProperty('output_enable')) {
        this.output_enable = initObj.output_enable
      }
      else {
        this.output_enable = false;
      }
      if (initObj.hasOwnProperty('default_output_value')) {
        this.default_output_value = initObj.default_output_value
      }
      else {
        this.default_output_value = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type GpioPinConfiguration
    // Serialize message field [pin_id]
    bufferOffset = _serializer.uint32(obj.pin_id, buffer, bufferOffset);
    // Serialize message field [pin_property]
    bufferOffset = _serializer.uint32(obj.pin_property, buffer, bufferOffset);
    // Serialize message field [output_enable]
    bufferOffset = _serializer.bool(obj.output_enable, buffer, bufferOffset);
    // Serialize message field [default_output_value]
    bufferOffset = _serializer.bool(obj.default_output_value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type GpioPinConfiguration
    let len;
    let data = new GpioPinConfiguration(null);
    // Deserialize message field [pin_id]
    data.pin_id = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [pin_property]
    data.pin_property = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [output_enable]
    data.output_enable = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [default_output_value]
    data.default_output_value = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 10;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/GpioPinConfiguration';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4b71615f4756e2920864ba411102db09';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 pin_id
    uint32 pin_property
    bool output_enable
    bool default_output_value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new GpioPinConfiguration(null);
    if (msg.pin_id !== undefined) {
      resolved.pin_id = msg.pin_id;
    }
    else {
      resolved.pin_id = 0
    }

    if (msg.pin_property !== undefined) {
      resolved.pin_property = msg.pin_property;
    }
    else {
      resolved.pin_property = 0
    }

    if (msg.output_enable !== undefined) {
      resolved.output_enable = msg.output_enable;
    }
    else {
      resolved.output_enable = false
    }

    if (msg.default_output_value !== undefined) {
      resolved.default_output_value = msg.default_output_value;
    }
    else {
      resolved.default_output_value = false
    }

    return resolved;
    }
};

module.exports = GpioPinConfiguration;
