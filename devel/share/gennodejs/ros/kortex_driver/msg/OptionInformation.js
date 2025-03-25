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

class OptionInformation {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.sensor = null;
      this.option = null;
      this.supported = null;
      this.read_only = null;
      this.minimum = null;
      this.maximum = null;
      this.step = null;
      this.default_value = null;
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
      if (initObj.hasOwnProperty('supported')) {
        this.supported = initObj.supported
      }
      else {
        this.supported = false;
      }
      if (initObj.hasOwnProperty('read_only')) {
        this.read_only = initObj.read_only
      }
      else {
        this.read_only = false;
      }
      if (initObj.hasOwnProperty('minimum')) {
        this.minimum = initObj.minimum
      }
      else {
        this.minimum = 0.0;
      }
      if (initObj.hasOwnProperty('maximum')) {
        this.maximum = initObj.maximum
      }
      else {
        this.maximum = 0.0;
      }
      if (initObj.hasOwnProperty('step')) {
        this.step = initObj.step
      }
      else {
        this.step = 0.0;
      }
      if (initObj.hasOwnProperty('default_value')) {
        this.default_value = initObj.default_value
      }
      else {
        this.default_value = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type OptionInformation
    // Serialize message field [sensor]
    bufferOffset = _serializer.uint32(obj.sensor, buffer, bufferOffset);
    // Serialize message field [option]
    bufferOffset = _serializer.uint32(obj.option, buffer, bufferOffset);
    // Serialize message field [supported]
    bufferOffset = _serializer.bool(obj.supported, buffer, bufferOffset);
    // Serialize message field [read_only]
    bufferOffset = _serializer.bool(obj.read_only, buffer, bufferOffset);
    // Serialize message field [minimum]
    bufferOffset = _serializer.float32(obj.minimum, buffer, bufferOffset);
    // Serialize message field [maximum]
    bufferOffset = _serializer.float32(obj.maximum, buffer, bufferOffset);
    // Serialize message field [step]
    bufferOffset = _serializer.float32(obj.step, buffer, bufferOffset);
    // Serialize message field [default_value]
    bufferOffset = _serializer.float32(obj.default_value, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type OptionInformation
    let len;
    let data = new OptionInformation(null);
    // Deserialize message field [sensor]
    data.sensor = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [option]
    data.option = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [supported]
    data.supported = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [read_only]
    data.read_only = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [minimum]
    data.minimum = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [maximum]
    data.maximum = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [step]
    data.step = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [default_value]
    data.default_value = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 26;
  }

  static datatype() {
    // Returns string type for a message object
    return 'kortex_driver/OptionInformation';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4a9212bde4d95812b702827b151ff2e7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    uint32 sensor
    uint32 option
    bool supported
    bool read_only
    float32 minimum
    float32 maximum
    float32 step
    float32 default_value
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new OptionInformation(null);
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

    if (msg.supported !== undefined) {
      resolved.supported = msg.supported;
    }
    else {
      resolved.supported = false
    }

    if (msg.read_only !== undefined) {
      resolved.read_only = msg.read_only;
    }
    else {
      resolved.read_only = false
    }

    if (msg.minimum !== undefined) {
      resolved.minimum = msg.minimum;
    }
    else {
      resolved.minimum = 0.0
    }

    if (msg.maximum !== undefined) {
      resolved.maximum = msg.maximum;
    }
    else {
      resolved.maximum = 0.0
    }

    if (msg.step !== undefined) {
      resolved.step = msg.step;
    }
    else {
      resolved.step = 0.0
    }

    if (msg.default_value !== undefined) {
      resolved.default_value = msg.default_value;
    }
    else {
      resolved.default_value = 0.0
    }

    return resolved;
    }
};

module.exports = OptionInformation;
